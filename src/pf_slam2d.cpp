/*
 * IRIS Localization and Mapping (LaMa)
 *
 * Copyright (c) 2019-today, Eurico Pedrosa, University of Aveiro - Portugal
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Aveiro nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <fstream>

#include "lama/print.h"
#include "lama/thread_pool.h"

#include "lama/random.h"

#include "lama/nlls/gauss_newton.h"
#include "lama/nlls/levenberg_marquardt.h"

#include "lama/pf_slam2d.h"
#include "lama/match_surface_2d.h"

#include "lama/sdm/export.h"

lama::PFSlam2D::Options::Options()
{
    particles = 30;
    use_gaussian_proposal = false;
    srr = 0.1;
    srt = 0.2;
    str = 0.1;
    stt = 0.2;
    meas_sigma = 0.05;
    meas_sigma_gain = 3;

    trans_thresh    = 0.5;
    rot_thresh      = 0.5;
    l2_max          = 0.5;
    truncated_ray   = -1;
    resolution      = 0.05;
    patch_size      = 32;
    max_iter        = 100;

    threads = -1;
    use_compression = false;
    cache_size = 60;
}

lama::PFSlam2D::PFSlam2D(const Options& options)
    : options_(options)
{
    /* solver_options_.write_to_stdout= true; */
    solver_options_.max_iterations = options.max_iter;
    solver_options_.strategy       = makeStrategy(options.strategy, Vector2d::Zero());
    /* solver_options_.robust_cost    = makeRobust("cauchy", 0.25); */
    solver_options_.robust_cost.reset(new CauchyWeight(0.15));

    has_first_scan = false;
    truncated_ray_ = options.truncated_ray;

    acc_trans_ = 0.0;
    acc_rot_   = 0.0;

    delta_free_ = options_.resolution * std::sqrt(2.0);

    if (options_.threads <= 1){
        thread_pool_ = 0;
    } else {
        thread_pool_ = new ThreadPool;
        thread_pool_->init(options_.threads);
    }

    // handle rng seed
    if (options_.seed == 0)
        options_.seed = random::genSeed();

    random::setSeed(options_.seed);
}

lama::PFSlam2D::~PFSlam2D()
{
    delete thread_pool_;
}


void lama::PFSlam2D::setPrior(const Pose2D& prior)
{
    pose_ = prior;
}

uint64_t lama::PFSlam2D::getMemoryUsage() const
{
    uint64_t total = 0;

    const uint32_t num_particles = options_.particles;
    for (uint32_t i = 0; i < num_particles; ++i){
        total += particles_[current_particle_set_][i].dm->memory();
        total += particles_[current_particle_set_][i].occ->memory();
    }

    return total;
}

uint64_t lama::PFSlam2D::getMemoryUsage(uint64_t& occmem, uint64_t& dmmem) const
{
    occmem = 0;
    dmmem  = 0;

    const uint32_t num_particles = options_.particles;
    for (uint32_t i = 0; i < num_particles; ++i){
        occmem += particles_[current_particle_set_][0].occ->memory();
        dmmem  += particles_[current_particle_set_][0].dm->memory();
    }

    return occmem + dmmem;
}

bool lama::PFSlam2D::update(const PointCloudXYZ::Ptr& surface, const Pose2D& odometry, double timestamp)
{
    current_surface_ = surface;

    if (not has_first_scan){
        odom_ = odometry;
        timestamps_.push_back(timestamp);

        // initialize particles
        const uint32_t num_particles = options_.particles;
        particles_[0].resize(num_particles);
        current_particle_set_ = 0;

        particles_[0][0].poses.push_back(pose_);
        particles_[0][0].pose = pose_;

        particles_[0][0].weight     = 0.0;
        particles_[0][0].weight_sum = 0.0;
        particles_[0][0].dm = DynamicDistanceMapPtr(new DynamicDistanceMap(options_.resolution, options_.patch_size));
        particles_[0][0].dm->setMaxDistance(options_.l2_max);
        particles_[0][0].dm->useCompression(options_.use_compression,  options_.cache_size, options_.calgorithm);

        particles_[0][0].occ = FrequencyOccupancyMapPtr(new FrequencyOccupancyMap(options_.resolution, options_.patch_size));
        particles_[0][0].occ->useCompression(options_.use_compression, options_.cache_size, options_.calgorithm);

        updateParticleMaps(&(particles_[0][0]));

        for (uint32_t i = 1; i < num_particles; ++i){
            particles_[0][i].poses.push_back(pose_);
            particles_[0][i].pose = pose_;

            particles_[0][i].weight     = 0.0;
            particles_[0][i].weight_sum = 0.0;
            particles_[0][i].dm  = DynamicDistanceMapPtr(new DynamicDistanceMap(*particles_[0][0].dm ));
            particles_[0][i].occ = FrequencyOccupancyMapPtr(new FrequencyOccupancyMap(*particles_[0][0].occ));
        }

        has_first_scan = true;
        return true;
    }

    // 1. Predict from odometry
    Pose2D odelta = odom_ - odometry;
    odom_ = odometry;

    const uint32_t num_particles = options_.particles;
    for (uint32_t i = 0; i < num_particles; ++i)
        drawFromMotion(odelta, particles_[current_particle_set_][i].pose, particles_[current_particle_set_][i].pose);

    // only continue if the necessary motion was gathered.
    acc_trans_ += odelta.xy().norm();
    acc_rot_   += std::fabs(odelta.rotation());
    if (acc_trans_ <= options_.trans_thresh &&
        acc_rot_ <= options_.rot_thresh)
        return false;

    // save the reading
    //readings_.push_back(surface);

    acc_trans_ = 0;
    acc_rot_   = 0;

    // 2. Apply scan matching
    if (thread_pool_){

        for (uint32_t i = 0; i < num_particles; ++i)
            thread_pool_->enqueue([this, i](){
                scanMatch(&(particles_[current_particle_set_][i]));
            });

        thread_pool_->wait();
    } else {
        for (uint32_t i = 0; i < num_particles; ++i){
            scanMatch(&particles_[current_particle_set_][i]);
        } // end for
    } // end if

    // 3. Normalize weights and calculate neff
    normalize();

    // 4. resample if needed
    if (neff_ < (options_.particles*0.5))
        resample();

    // 3. Update maps
    if (thread_pool_){
        for (uint32_t i = 0; i < num_particles; ++i)
            thread_pool_->enqueue([this, i](){
                updateParticleMaps(&(particles_[current_particle_set_][i]));
            });

        thread_pool_->wait();
    } else {
        for (uint32_t i = 0; i < num_particles; ++i)
            updateParticleMaps(&particles_[current_particle_set_][i]);
    }

    return true;
}

size_t lama::PFSlam2D::getBestParticleIdx() const
{
    const uint32_t num_particles = options_.particles;

    size_t best_idx = 0;
    double best_ws  = particles_[current_particle_set_][0].weight_sum;

    for (uint32_t i = 1; i < num_particles; ++i){

        if (best_ws < particles_[current_particle_set_][i].weight_sum){
            best_ws = particles_[current_particle_set_][i].weight_sum;
            best_idx = i;
        }
    }

    return best_idx;
}

lama::Pose2D lama::PFSlam2D::getPose() const
{
    size_t pidx = getBestParticleIdx();
    return particles_[current_particle_set_][pidx].pose;
}

void lama::PFSlam2D::saveOccImage(const std::string& name) const
{
    size_t pidx = getBestParticleIdx();
    sdm::export_to_png(*particles_[current_particle_set_][pidx].occ, name);
}

lama::PFSlam2D::StrategyPtr lama::PFSlam2D::makeStrategy(const std::string& name, const VectorXd& parameters)
{
    if (name == "gn"){
        return StrategyPtr(new GaussNewton);
    }else {
        return StrategyPtr(new LevenbergMarquard);
    }
}

lama::PFSlam2D::RobustCostPtr lama::PFSlam2D::makeRobust(const std::string& name, const double& param)
{
    if (name == "cauchy")
        return RobustCostPtr(new CauchyWeight(0.25));
    else if (name == "tstudent")
        return RobustCostPtr(new TDistributionWeight(3));
    else if (name == "tukey")
        return RobustCostPtr(new TukeyWeight);
    else
        return RobustCostPtr(new UnitWeight);
}

void lama::PFSlam2D::drawFromMotion(const Pose2D& delta, const Pose2D& old_pose, Pose2D& pose)
{
    double sigma, x, y, yaw;
    double sxy = 0.3 * options_.srr;

    sigma = options_.srr * std::fabs(delta.x())   +
            options_.str * std::fabs(delta.rotation()) +
            sxy * std::fabs(delta.y());

    x = delta.x() + random::normal(sigma);

    sigma = options_.srr * std::fabs(delta.y())   +
            options_.str * std::fabs(delta.rotation()) +
            sxy * std::fabs(delta.x());

    y = delta.y() + random::normal(sigma);

    sigma = options_.stt * std::fabs(delta.rotation()) +
            options_.srt * delta.xy().norm();

    yaw = delta.rotation() + random::normal(sigma);
    yaw = std::fmod(yaw, 2*M_PI);
    if (yaw > M_PI)
        yaw -= 2*M_PI;

    pose += Pose2D(x, y, yaw);
}

double lama::PFSlam2D::calculateLikelihood(const Particle& particle)
{
    PointCloudXYZ::Ptr surface = current_surface_;

    Affine3d moving_tf = Translation3d(surface->sensor_origin_) * surface->sensor_orientation_;

    Vector3d trans;
    trans << particle.pose.x(), particle.pose.y(), 0.0;

    Affine3d fixed_tf = Translation3d(trans) * AngleAxisd(particle.pose.rotation(), Vector3d::UnitZ());

    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
    const size_t num_points = surface->points.size();
    //== transform point cloud
    Affine3d tf = fixed_tf * moving_tf;
    cloud->points.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i)
        cloud->points.push_back(tf * surface->points[i]);
    //==

    double likelihood = 0;
    for (size_t i = 0; i < num_points; ++i){
        Vector3d hit = cloud->points[i];
        double dist = particle.dm->distance(hit, 0);
        likelihood += - (dist*dist) / options_.meas_sigma;
    } // end for

    return likelihood;
}

void lama::PFSlam2D::scanMatch(Particle* particle)
{
    /* const PointCloudXYZ::Ptr surface = readings_.back(); */
    const PointCloudXYZ::Ptr surface = current_surface_;

    MatchSurface2D match_surface(particle->dm.get(), surface, particle->pose.state);

    SolverOptions so;
    so.max_iterations = options_.max_iter;
    /* so.strategy       = makeStrategy(options_.strategy, Vector2d::Zero()); */
    so.strategy.reset(new GaussNewton);
    so.robust_cost.reset(new CauchyWeight(0.15));

    Solve(so, match_surface, 0);
    particle->pose.state = match_surface.getState();

    particle->poses.push_back(particle->pose);

    double l = calculateLikelihood(*particle);
    particle->weight_sum += l;
    particle->weight     += l;
}

void lama::PFSlam2D::updateParticleMaps(Particle* particle)
{
    const PointCloudXYZ::Ptr surface = current_surface_;

    // 1. Transform the point cloud to the model coordinates.
    Affine3d moving_tf = Translation3d(surface->sensor_origin_) * surface->sensor_orientation_;
    Affine3d fixed_tf  = Translation3d(particle->pose.x(), particle->pose.y(), 0.0) * AngleAxisd(particle->pose.rotation(), Vector3d::UnitZ());

    // the sensor origin (in map coordinates) is the origin
    // for the ray casting.
    Vector3d wso = (fixed_tf * moving_tf).translation();

    const size_t num_points = surface->points.size();
    Affine3d tf = fixed_tf * moving_tf;

    // 2. generate the free and occupied positions.
    VectorVector3ui free;

    // generate the ray casts
    for (size_t i = 0; i < num_points; ++i){
        Vector3d start = wso;
        Vector3d hit   = tf * surface->points[i];

        if (truncated_ray_ > 0.0){
            Vector3d AB = hit - wso;
            double truncate_size = std::min(AB.norm(), truncated_ray_);
            start = hit - AB.normalized() * truncate_size;
        }

        Vector3ui mhit = particle->occ->w2m(hit);

        bool changed = particle->occ->setOccupied(mhit);
        if ( changed ) particle->dm->addObstacle(mhit);

        particle->occ->computeRay(particle->occ->w2m(start), mhit, free);
    }

    const size_t num_free = free.size();
    for (size_t i = 0; i < num_free; ++i){
        bool changed = particle->occ->setFree(free[i]);
        if ( changed ) particle->dm->removeObstacle(free[i]);
    }

    // 3. Update the distance map
    particle->dm->update();
}

void lama::PFSlam2D::normalize()
{
    /* double gain = options_.meas_sigma_gain; //1.0 / (options_.meas_sigma_gain * options_.particles); */
    double gain = 1.0 / (options_.meas_sigma_gain * options_.particles);
    double max_l  = particles_[current_particle_set_][0].weight;
    const uint32_t num_particles = options_.particles;
    for (uint32_t i = 1; i < num_particles; ++i)
        if (max_l < particles_[current_particle_set_][i].weight)
            max_l = particles_[current_particle_set_][i].weight;

    double sum = 0;
    for (uint32_t i = 0; i < num_particles; ++i){

        particles_[current_particle_set_][i].normalized_weight = std::exp(gain*(particles_[current_particle_set_][i].weight - max_l));
        sum += particles_[current_particle_set_][i].normalized_weight;
    }

    neff_ = 0;
    for (uint32_t i = 0; i < num_particles; ++i){
        particles_[current_particle_set_][i].normalized_weight /= sum;
        neff_ += particles_[current_particle_set_][i].normalized_weight * particles_[current_particle_set_][i].normalized_weight;
    }

    neff_ = 1.0 / neff_;
}

void lama::PFSlam2D::resample()
{
    const uint32_t num_particles = options_.particles;
    std::vector<int32_t> sample_idx(num_particles);

    double interval = 1.0 / (double)num_particles;

    double target = interval * random::uniform();
    double   cw  = 0.0;
    uint32_t n   = 0;
    for (size_t i = 0; i < num_particles; ++i){
        cw += particles_[current_particle_set_][i].normalized_weight;

        while( cw > target){
            sample_idx[n++]=i;
            target += interval;
        }
    }

    // generate a new set of particles

    uint8_t ps = 1 - current_particle_set_;
    particles_[ps].resize(num_particles);

    for (size_t i = 0; i < num_particles; ++i) {
        uint32_t idx = sample_idx[i];;

        particles_[ps][i] = particles_[current_particle_set_][ idx ];
        particles_[ps][i].weight  = 0.0;
        particles_[ps][i].weight_sum  = particles_[current_particle_set_][idx].weight_sum;

        particles_[ps][i].dm  = DynamicDistanceMapPtr(new DynamicDistanceMap(*(particles_[current_particle_set_][idx].dm)));
        particles_[ps][i].occ = FrequencyOccupancyMapPtr(new FrequencyOccupancyMap(*(particles_[current_particle_set_][idx].occ)));
    }

    particles_[current_particle_set_].clear();
    current_particle_set_ = ps;
}

