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
#include "lama/time.h"
#include "lama/aabb.h"

#include "lama/nlls/gauss_newton.h"
#include "lama/nlls/levenberg_marquardt.h"

#include "lama/slam2d.h"
#include "lama/match_surface_2d.h"

std::string lama::Slam2D::Summary::report() const
{
    std::string report = format("\n LaMa Slam2D - Report\n"
                                " ====================\n");

    Eigen::Map<const VectorXd> v_t(&time[0], time.size());
    Eigen::Map<const VectorXd> v_ts(&time_solving[0], time_solving.size());
    Eigen::Map<const VectorXd> v_tm(&time_mapping[0], time_mapping.size());
    Eigen::Map<const VectorXd> v_mem(&memory[0], memory.size());

    auto time_span = v_t.sum();
    auto time_mean = v_t.mean();

    auto stampdiff = timestamp.back() - timestamp.front();

    report += format(" Number of updates     %ld\n"
                     " Max memory usage      %.2f MiB\n"
                     " Problem time span     %d minute(s) and %d second(s)\n"
                     " Execution time span   %d minute(s) and %d second(s)\n"
                     " Execution frequency   %.2f Hz\n"
                     " Realtime factor       %.2fx\n",
                     time.size(), v_mem.maxCoeff() / 1024.0 / 1024.0,
                     ((uint32_t)stampdiff) / 60, ((uint32_t)stampdiff) % 60,
                     ((uint32_t)time_span) / 60, ((uint32_t)time_span) % 60,
                     (1.0 / time_mean), stampdiff / time_span );


    auto v_t_std  = std::sqrt((v_t.array() - v_t.mean()).square().sum()/(v_t.size()-1));
    auto v_ts_std = std::sqrt((v_ts.array() - v_ts.mean()).square().sum()/(v_ts.size()-1));
    auto v_tm_std = std::sqrt((v_tm.array() - v_tm.mean()).square().sum()/(v_tm.size()-1));

    report += format("\n Execution time (mean ± std [min, max]) in milliseconds\n"
                     " --------------------------------------------------------\n"
                     " Update          %f ± %f [%f, %f]\n"
                     "   Optimization  %f ± %f [%f, %f]\n"
                     "   Mapping       %f ± %f [%f, %f]\n",
                     v_t.mean() * 1000.0, v_t_std * 1000.0,
                     v_t.minCoeff() * 1000.0, v_t.maxCoeff() * 1000.0,
                     v_ts.mean() * 1000.0, v_ts_std * 1000.0,
                     v_ts.minCoeff() * 1000.0, v_ts.maxCoeff() * 1000.0,
                     v_tm.mean() * 1000.0, v_tm_std * 1000.0,
                     v_tm.minCoeff() * 1000.0, v_tm.maxCoeff() * 1000.0);

    return report;
}

lama::Slam2D::Slam2D(const Options& options)
{
    distance_map_ = new DynamicDistanceMap(options.resolution, options.patch_size);
    distance_map_->setMaxDistance(options.l2_max);

    occupancy_map_ = new FrequencyOccupancyMap(options.resolution, options.patch_size);

    distance_map_->useCompression(options.use_compression,  options.cache_size, options.calgorithm);
    occupancy_map_->useCompression(options.use_compression, options.cache_size, options.calgorithm);

    /* solver_options_.write_to_stdout= true; */
    solver_options_.max_iterations = options.max_iter;
    solver_options_.strategy       = makeStrategy(options.strategy);
    /* solver_options_.robust_cost    = makeRobust("cauchy", 0.25); */
    solver_options_.robust_cost.reset(new CauchyWeight(0.15));

    trans_thresh_ = options.trans_thresh;
    rot_thresh_   = options.rot_thresh;

    has_first_scan = false;
    number_of_proccessed_cells_ = 0;
    truncated_ray_ = options.truncated_ray;
    truncated_range_ = options.truncated_range;

    transient_map_ = options.transient_map;

    if (options.create_summary)
        summary = new Summary();

}

lama::Slam2D::~Slam2D()
{
    delete distance_map_;
    delete occupancy_map_;
    delete summary;
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
    Timer timer(true);

    if (not has_first_scan){
        odom_ = odometry;
        updateMaps(surface);

        if (summary){
            auto elapsed = timer.elapsed();
            probeStamp(timestamp);
            probeTime(elapsed);
            probeMTime(elapsed);
            probeMem();
        }

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
    Timer time_solving(true);

    MatchSurface2D match_surface(distance_map_, surface, pose_.state );
    Solve(solver_options_, match_surface, 0);

    pose_.state = match_surface.getState();
    if (summary)
        probeSTime(time_solving.elapsed());

    // 3. Update maps
    Timer time_mapping(true);
    updateMaps(surface);

    if (summary){
        probeMTime(time_mapping.elapsed());
        probeTime(timer.elapsed());

        probeStamp(timestamp);
        probeMem();
    }

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

lama::Slam2D::StrategyPtr lama::Slam2D::makeStrategy(const std::string& name)
{
    if (name == "lm"){
        return StrategyPtr(new LevenbergMarquard);
    }else {
        return StrategyPtr(new GaussNewton);
    }
}

lama::Slam2D::RobustCostPtr lama::Slam2D::makeRobust(const std::string& name)
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

    // This will be used to calculate the surface AABB.
    Vector3d min, max;
    min.fill(std::numeric_limits<Vector3d::Scalar>::max());
    max.fill(-std::numeric_limits<Vector3d::Scalar>::max());

    Dictionary<uint64_t, bool> seen;

    // generate the ray casts
    for (size_t i = 0; i < num_points; ++i){
        Vector3d start = wso;
        Vector3d hit = tf * surface->points[i];
        Vector3d AB;
        double ray_length = 1.0; // this will be overwritten but gcc fails to notice.
        bool mark_hit = true;

        // Attempt to truncate the ray if it is larger than the truncated range
        if (truncated_range_ > 0.0){
            AB = hit - start;
            ray_length = AB.norm();
            if (truncated_range_ < ray_length){
                // Truncate the hit point and choose not to mark an obstacle for it
                hit = start + AB / ray_length * truncated_range_;
                mark_hit = false;
            }
        }// end if

        // Only attempt to truncate a ray if the hit should be marked. If the hit
        // should not be marked then the range has already been truncated
        if (mark_hit and (truncated_ray_ > 0.0)) {
            // Avoid computing the AB vector again if it has already been calculated
            if (truncated_range_ == 0.0) {
                AB = hit - start;
                ray_length = AB.norm();
            }// end if

            if (truncated_ray_ < ray_length)
                start = hit - AB / ray_length * truncated_ray_;
        }

        Vector3ui mhit = occupancy_map_->w2m(hit);
        if (transient_map_){
            min = min.cwiseMin(hit);
            max = max.cwiseMax(hit);
        }

        if (mark_hit){
            bool changed = occupancy_map_->setOccupied(mhit);
            if ( changed ) distance_map_->addObstacle(mhit);
        }

        occupancy_map_->computeRay(occupancy_map_->w2m(start), mhit,
        [this](const Vector3ui& coord){
            bool changed = occupancy_map_->setFree(coord);
            if ( changed ) distance_map_->removeObstacle(coord);
        });
    }

    // 3. Update the distance map
    number_of_proccessed_cells_ = distance_map_->update();

    // 4. Transient map (if enabled)
    // When this option is enabled, we only keep the most recent portion of the
    // map "sensed" by the latest surface. To do that, we create an axis aligned
    // bounding box (AABB) of the surface and test it for intersection with the
    // AABB of each existing patch. Patches with AABBs that do not intersect with
    // the surface AABB are deleted.
    if (not transient_map_) return;

    // make sure the Z coordinate is zero (2D remember?).
    min(2) = max(2) = 0;

    // make sure the min and max are at the same distance from the robot's pose.
    double xdist = std::max(pose_.x() - min(0), max(0) - pose_.x()) * 2.0;
    double ydist = std::max(pose_.y() - min(1), max(1) - pose_.y()) * 2.0;

    min(0) = pose_.x() - xdist;
    min(1) = pose_.y() - ydist;

    max(0) = pose_.x() + xdist;
    max(1) = pose_.y() + ydist;

    // Axis Aligned Bounding box of the current surface (or lidar scan).
    AABB a(min, max);

    // expand the AABB by twice the maximum distance of distance map.
    a.hwidth.array() += 2.0 * distance_map_->maxDistance();

    // We use the distance map for intersection test because its bounds are larger than
    // the occupancy map bounds. This way, we make sure all unecessary patches are removed.
    VectorVector3ui to_remove;
    distance_map_->visit_all_patches([&a, &to_remove, this](auto& origin){
        const uint32_t length = this->occupancy_map_->patch_length;

        Vector3d ws = this->occupancy_map_->m2w(origin);
        Vector3d we = this->occupancy_map_->m2w(origin + Vector3ui(length, length, 0.0) );

        ws(2) = we(2) = 0.0;
        AABB b(ws, we);

        bool intersect = a.testIntersection(b);
        if ( intersect ) return;

        // It is not a good idea to remove an item from a container while iterating over it.
        // Therefore, we push the patch origin so we can delete it afterwards.
        to_remove.push_back(origin);
    });

    for (auto& coord : to_remove){
        occupancy_map_->deletePatchAt(coord);
        distance_map_->deletePatchAt(coord);
    }
}
