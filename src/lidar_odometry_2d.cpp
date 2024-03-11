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

#include "lama/aabb.h"
#include "lama/nlls/gauss_newton.h"
#include "lama/match_surface_2d.h"

#include "lama/lidar_odometry_2d.h"

lama::LidarOdometry2D::LidarOdometry2D(const Options& options)
{
    distance_map = new DynamicDistanceMap(options.resolution);
    distance_map->setMaxDistance(1.0);
    occupancy_map = new ProbabilisticOccupancyMap(options.resolution);

    solver_options.max_iterations = options.max_iter;
    solver_options.strategy.reset(new GaussNewton);
    solver_options.robust_cost.reset(new CauchyWeight(0.15));
}

lama::LidarOdometry2D::~LidarOdometry2D()
{
    delete distance_map;
    delete occupancy_map;
}

bool lama::LidarOdometry2D::update(const PointCloudXYZ::Ptr& surface, double /*timestamp*/)
{
    if (not has_first_scan) {
        updateMaps(surface);
        has_first_scan = true;
        return true;
    }

    MatchSurface2D match_surface(distance_map, surface, odom.state);
    Solve(solver_options, match_surface, 0);

    // Update the odometry to the new estimate.
    odom.state = match_surface.getState();

    // Only update the maps after the robot moves 0.1 meters or 0.5 rads.
    Pose2D odelta = map_update_odom - odom;
    if (odelta.xy().norm() > 0.1 || std::abs(odelta.rotation()) > 0.5){
        updateMaps(surface);
        map_update_odom = odom;
    }

    return true;
}

void lama::LidarOdometry2D::updateMaps(const PointCloudXYZ::Ptr& surface)
{
    // 1. Transform the point cloud to the model coordinates.
    Affine3d moving_tf = Translation3d(surface->sensor_origin_) * surface->sensor_orientation_;
    Affine3d fixed_tf = Translation3d(odom.x(), odom.y(), 0.0) * AngleAxisd(odom.rotation(), Vector3d::UnitZ());

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

    // generate the ray casts
    for (size_t i = 0; i < num_points; ++i){
        Vector3d start = wso;
        Vector3d hit = tf * surface->points[i];

        Vector3d AB = hit - start;
        double ray_length = AB.norm();

        if (ray_length >= 1.0)
            start = hit - AB / ray_length;

        Vector3ui mhit = occupancy_map->w2m(hit);
        min = min.cwiseMin(hit);
        max = max.cwiseMax(hit);

        bool changed = occupancy_map->setOccupied(mhit);
        if ( changed ) distance_map->addObstacle(mhit);

        occupancy_map->computeRay(occupancy_map->w2m(start), mhit,
        [this](const Vector3ui& coord){
            bool changed = occupancy_map->setFree(coord);
            if ( changed ) distance_map->removeObstacle(coord);
        });
    }

    // 3. Update the distance map
    distance_map->update();

    // 4. Transient map
    // When this option is enabled, we only keep the most recent portion of the
    // map "sensed" by the latest surface. To do that, we create an axis aligned
    // bounding box (AABB) of the surface and test it for intersection with the
    // AABB of each existing patch. Patches with AABBs that do not intersect with
    // the surface AABB are deleted.

    // make sure the Z coordinate is zero (2D remember?).
    min(2) = max(2) = 0;

    // make sure the min and max are at the same distance from the robot's pose.
    double xdist = std::max(odom.x() - min(0), max(0) - odom.x());
    double ydist = std::max(odom.y() - min(1), max(1) - odom.y());

    min(0) = odom.x() - xdist;
    min(1) = odom.y() - ydist;

    max(0) = odom.x() + xdist;
    max(1) = odom.y() + ydist;

    // Axis Aligned Bounding box of the current surface (or lidar scan).
    AABB a(min, max);

    // expand the AABB by twice the maximum distance of distance map.
    a.hwidth.array() += 2.0 * distance_map->maxDistance();

    // We use the distance map for intersection test because its bounds are larger than
    // the occupancy map bounds. This way, we make sure all unecessary patches are removed.
    VectorVector3ui to_remove;
    distance_map->visit_all_patches([&a, &to_remove, this](auto& origin){
        const uint32_t length = this->occupancy_map->patch_length;

        Vector3d ws = this->occupancy_map->m2w(origin);
        Vector3d we = this->occupancy_map->m2w(origin + Vector3ui(length, length, 0.0) );

        ws(2) = we(2) = 0.0;
        AABB b(ws, we);

        bool intersect = a.testIntersection(b);
        if ( intersect ) return;

        // It is not a good idea to remove an item from a container while iterating over it.
        // Therefore, we push the patch origin so we can delete it afterwards.
        to_remove.push_back(origin);
    });

    for (auto& coord : to_remove){
        occupancy_map->deletePatchAt(coord);
        distance_map->deletePatchAt(coord);
    }
}
