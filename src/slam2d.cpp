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

#include "lama/nlls/gauss_newton.h"
#include "lama/nlls/levenberg_marquardt.h"

#include "lama/slam2d.h"
#include "lama/match_surface_2d.h"

lama::Slam2D::Slam2D(const Options& options)
{
    distance_map_ = new DynamicDistanceMap(options.resolution, options.patch_size);
    distance_map_->setMaxDistance(options.l2_max);

    occupancy_map_ = new FrequencyOccupancyMap(options.resolution, options.patch_size);

    distance_map_->useCompression(options.use_compression,  options.cache_size, options.calgorithm);
    occupancy_map_->useCompression(options.use_compression, options.cache_size, options.calgorithm);

    /* solver_options_.write_to_stdout= true; */
    solver_options_.max_iterations = options.max_iter;
    solver_options_.strategy       = makeStrategy(options.strategy, Vector2d::Zero());
    /* solver_options_.robust_cost    = makeRobust("cauchy", 0.25); */
    solver_options_.robust_cost.reset(new CauchyWeight(0.15));

    trans_thresh_ = options.trans_thresh;
    rot_thresh_   = options.rot_thresh;

    has_first_scan = false;
    number_of_proccessed_cells_ = 0;
    truncated_ray_ = options.truncated_ray;
}

lama::Slam2D::~Slam2D()
{
    delete distance_map_;
    delete occupancy_map_;
}

bool lama::Slam2D::enoughMotion(const Pose2D& odometry)
{
    if (not has_first_scan)
        return true;

    Pose2D odelta = odom_ - odometry;

    if (odelta.xy().norm() <= trans_thresh_ && std::abs(odelta.rotation()) <= rot_thresh_)
        return false;

    return true;
}

bool lama::Slam2D::update(const PointCloudXYZ::Ptr& surface, const Pose2D& odometry, double timestamp)
{
    if (not has_first_scan){
        odom_ = odometry;
        updateMaps(surface);

        has_first_scan = true;
        return true;
    }

    // 1. Predict from odometry
    Pose2D odelta = odom_ - odometry;
    Pose2D ppose  = pose_ + odelta;

    // only continue if the necessary motion was gathered.
    if (odelta.xy().norm() <= trans_thresh_ &&
        std::abs(odelta.rotation()) <= rot_thresh_)
        return false;

    pose_ = ppose;
    odom_ = odometry;

    // 2. Optimize
    MatchSurface2D match_surface(distance_map_, surface, pose_.state );
    Solve(solver_options_, match_surface, 0);

    pose_.state = match_surface.getState();

    // 3. Update maps
    updateMaps(surface);

    return true;
}

uint64_t lama::Slam2D::getMemoryUsage() const
{
    uint64_t memory = 0;
    memory += occupancy_map_->memory();
    memory += distance_map_->memory();

    return memory;
}

uint64_t lama::Slam2D::getMemoryUsage(uint64_t& occmem, uint64_t& dmmem) const
{
    occmem = 0;
    dmmem  = 0;

    occmem = occupancy_map_->memory();
    dmmem  = distance_map_->memory();

    return occmem + dmmem;
}

void lama::Slam2D::useCompression(bool compression, const std::string& algorithm)
{
    distance_map_->useCompression(compression,  60, algorithm);
    occupancy_map_->useCompression(compression, 60, algorithm);
}

lama::Slam2D::StrategyPtr lama::Slam2D::makeStrategy(const std::string& name, const VectorXd& parameters)
{
    if (name == "lm"){
        return StrategyPtr(new LevenbergMarquard);
    }else {
        return StrategyPtr(new GaussNewton);
    }
}

lama::Slam2D::RobustCostPtr lama::Slam2D::makeRobust(const std::string& name, const double& param)
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

void lama::Slam2D::updateMaps(const PointCloudXYZ::Ptr& surface)
{
    // 1. Transform the point cloud to the model coordinates.
    Affine3d moving_tf = Translation3d(surface->sensor_origin_) * surface->sensor_orientation_;
    Affine3d fixed_tf = Translation3d(pose_.x(), pose_.y(), 0.0) * AngleAxisd(pose_.rotation(), Vector3d::UnitZ());

    // the sensor origin (in map coordinates) is the origin
    // for the ray casting.
    Vector3d wso = (fixed_tf * moving_tf).translation();
    /* Vector3ui so = occupancy_map_->w2m(wso); */

    const size_t num_points = surface->points.size();
    Affine3d tf = fixed_tf * moving_tf;

    // 2. generate the free and occupied positions.
    VectorVector3ui free;
    // generate the ray casts
    for (size_t i = 0; i < num_points; ++i){
        Vector3d start = wso;
        Vector3d hit = tf * surface->points[i];

        if (truncated_ray_ > 0.0){
            Vector3d AB = hit - wso;
            double truncate_size = std::min(AB.norm(), truncated_ray_);
            start = hit - AB.normalized() * truncate_size;
        }

        Vector3ui mhit = occupancy_map_->w2m(hit);

        bool changed = occupancy_map_->setOccupied(mhit);
        if ( changed ) distance_map_->addObstacle(mhit);

        occupancy_map_->computeRay(occupancy_map_->w2m(start), mhit, free);
    }

    const size_t num_free = free.size();
    for (size_t i = 0; i < num_free; ++i){
        bool changed = occupancy_map_->setFree(free[i]);
        if ( changed ) distance_map_->removeObstacle(free[i]);
    }

    // 3. Update the distance map
    number_of_proccessed_cells_ = distance_map_->update();
}

