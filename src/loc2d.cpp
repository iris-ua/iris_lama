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

#include <iostream>
#include <fstream>

#include "lama/print.h"
#include "lama/random.h"

#include "lama/match_surface_2d.h"
#include "lama/loc2d.h"

#include "lama/nlls/gauss_newton.h"
#include "lama/nlls/levenberg_marquardt.h"

lama::Loc2D::Options::Options()
{
    trans_thresh = 0.5;
    rot_thresh   = 0.5;
    l2_max       = 1.0;
    resolution   = 0.05;
    patch_size   = 32;
    gloc_particles = 3000;
    gloc_iters   = 10;
    gloc_thresh  = 0.15;
    max_iter     = 100;
    cov_blend    = 0.0;
}


void lama::Loc2D::Init(const Options& options)
{
    occupancy_map = new SimpleOccupancyMap(options.resolution, options.patch_size, false);
    distance_map  = new DynamicDistanceMap(options.resolution, options.patch_size, false);
    distance_map->setMaxDistance(options.l2_max);

    /* solver_options_.write_to_stdout= true; */
    solver_options_.max_iterations = options.max_iter;
    solver_options_.strategy       = makeStrategy(options.strategy, Vector2d::Zero());
    /* solver_options_.robust_cost    = makeRobust("cauchy", 0.25); */
    solver_options_.robust_cost.reset(new CauchyWeight(0.15));

    trans_thresh_ = options.trans_thresh;
    rot_thresh_   = options.rot_thresh;
    rmse_ = 0.0;

    cov_.setIdentity(); // maybe it should be higher??

    has_first_scan = false;
    do_global_localization_ = false;

    gloc_particles_ = options.gloc_particles;
    gloc_thresh_ = options.gloc_thresh;

    gloc_iters_ = options.gloc_iters;
    gloc_cur_iter_ = 0;

    // make sure it is a number between 0 and 1.
    cov_blend_ = std::max(std::min(options.cov_blend, 1.0), 0.0);

    // Generate a sampling steps cache, if needed.
    if (sampling_steps_.size() > 0) return;

    double sstep = distance_map->resolution; // sampling step
    sampling_steps_.push_back({0.0, 0.0});
    for (int i = 1; i <= 20; ++i){
        sampling_steps_.push_back({ i * sstep, 0.0});
        sampling_steps_.push_back({0.0, i * sstep});

        sampling_steps_.push_back({-i * sstep, 0.0});
        sampling_steps_.push_back({0.0,-i * sstep});

        sampling_steps_.push_back({ i * sstep, i * sstep});
        sampling_steps_.push_back({-i * sstep, i * sstep});
        sampling_steps_.push_back({ i * sstep,-i * sstep});
        sampling_steps_.push_back({-i * sstep,-i * sstep});
    }// end for
}

lama::Loc2D::~Loc2D()
{}

bool lama::Loc2D::enoughMotion(const Pose2D& odometry)
{
    if (not has_first_scan)
        return true;

    Pose2D odelta = odom_ - odometry;

    if (odelta.xy().norm() <= trans_thresh_ && std::abs(odelta.rotation()) <= rot_thresh_)
        return false;

    return true;
}

bool lama::Loc2D::update(const PointCloudXYZ::Ptr& surface, const Pose2D& odometry, double timestamp, bool force_update)
{
    if (not has_first_scan) {
        odom_ = odometry;

        has_first_scan = true;

        // Return here if the update should not be enforced
        if (not force_update)
            return true;

        MatchSurface2D match_surface(distance_map, surface, pose_.state);
        VectorXd residuals;
        match_surface.eval(residuals, 0);
        rmse_ = sqrt(residuals.squaredNorm()/((double)(surface->points.size() - 1)));
    }

    // 1. Predict from odometry
    Pose2D odelta = odom_ - odometry;
    Pose2D ppose  = pose_ + odelta;

    // Only continue if the necessary motion was gathered or if
    // the update should be enforced
    if (not force_update and not enoughMotion(odometry))
        return false;

    pose_ = ppose;
    odom_ = odometry;

    if (do_global_localization_){
        // Use a maximum number of global localizations to prevent an infinity loop.
        if (gloc_cur_iter_ < gloc_iters_){
            gloc_cur_iter_++;

            // pose_ is set by the calle.
            globalLocalization(surface);
        } else {
            do_global_localization_ = false;
            gloc_cur_iter_ = 0;
        }// end if
    }// end if

    // 2. Optimize
    MatrixXd cov;
    MatchSurface2D match_surface(distance_map, surface, pose_.state);
    Solve(solver_options_, match_surface, &cov);
    pose_.state = match_surface.getState();
    cov_ = cov;

    if (cov_blend_ > 0.0)
        addSamplingCovariance(surface);

    VectorXd residuals;
    match_surface.eval(residuals, 0);
    rmse_ = sqrt(residuals.squaredNorm()/((double)(surface->points.size() - 1)));

    if (do_global_localization_){
        // evalute the scan mathing
        if (rmse_ < gloc_thresh_){
            do_global_localization_ = false;
            gloc_cur_iter_ = 0;
        }// end if

    }

    return true;
}

void lama::Loc2D::triggerGlobalLocalization()
{
    do_global_localization_ = true;
}

void lama::Loc2D::addSamplingCovariance(const PointCloudXYZ::Ptr& surface)
{
    // Calculate covariance with sampling.
    double x, y;
    Matrix2d K = Matrix2d::Zero();
    Vector2d u = Vector2d::Zero();
    double s   = 0;

    // This one is static
    Affine3d moving_tf = Translation3d(surface->sensor_origin_) * surface->sensor_orientation_;

    const size_t num_points = surface->points.size();
    const size_t step = std::max(num_points / 100, 1ul);

    auto aa = AngleAxisd(pose_.rotation(), Vector3d::UnitZ());
    for (size_t i = 0; i < sampling_steps_.size(); ++i){
        x = pose_.x() + sampling_steps_[i].x();
        y = pose_.y() + sampling_steps_[i].y();

        Vector3d trans = (Vector3d() << x, y, 0.0).finished();

        Affine3d fixed_tf = Translation3d(trans) * aa;
        Affine3d tf = fixed_tf * moving_tf;

        double l = 0.0;
        for (size_t i = 0; i < num_points; i += step ){
            Vector3d hit = tf * surface->points[i];

            // Get distance without interpolation. It is faster and is enough.
            double dist = distance_map->distance(distance_map->w2m(hit));
            double e = std::exp( - (dist * dist) / 0.01);
            l += e*e*e;
        }// end for

        K = K + trans.head<2>() * trans.head<2>().transpose() * l;
        u = u + trans.head<2>() * l;
        s = s + l;
    }

    // E. B. Olson, “Real-time correlative scan matching,”
    // in 2009 IEEE International Conference on Robotics and Automation,
    // Kobe, May 2009, pp. 4387–4393. doi: 10.1109/ROBOT.2009.5152375.
    MatrixXd sampling_cov = Matrix2d::Zero();
    sampling_cov = (1.0 / s) * K - (1.0 / (s*s)) * u * u.transpose();

    // Linear interpolation of the covariances.
    const double& alpha = cov_blend_;
    cov_.topLeftCorner<2,2>() = alpha*sampling_cov + (1.0 - alpha)*cov_.topLeftCorner<2,2>();
}

void lama::Loc2D::globalLocalization(const PointCloudXYZ::Ptr& surface)
{
    Vector3d min, max;
    occupancy_map->bounds(min, max);

    Vector3d diff = max - min;

    double best_error = std::numeric_limits<double>::max();

    for (uint32_t i = 0; i < gloc_particles_; ++i){

        double x, y, a;

        for (;;){
            x = min[0] + random::uniform() * diff[0];
            y = min[1] + random::uniform() * diff[1];

            if (not occupancy_map->isFree(Vector3d(x, y, 0.0)))
                continue;

            a = random::uniform() * 2 * M_PI - M_PI;
            break;
        }

        Pose2D p(x, y , a);
        VectorXd residuals;
        MatchSurface2D match_surface(distance_map, surface, p.state);

        match_surface.eval(residuals, 0);

        double error = residuals.squaredNorm();
        if ( error < best_error ){
            best_error = error;
            pose_ = p;
        }
    } // end for

}

lama::Loc2D::StrategyPtr lama::Loc2D::makeStrategy(const std::string& name, const VectorXd& parameters)
{
    if (name == "lm")
        return StrategyPtr(new LevenbergMarquard);

    return StrategyPtr(new GaussNewton);
}

lama::Loc2D::RobustCostPtr lama::Loc2D::makeRobust(const std::string& name, const double& param)
{
    if (name == "cauchy")
        return RobustCostPtr(new CauchyWeight(0.15));
    else if (name == "tstudent")
        return RobustCostPtr(new TDistributionWeight(3));
    else if (name == "tukey")
        return RobustCostPtr(new TukeyWeight);
    else
        return RobustCostPtr(new UnitWeight);
}


