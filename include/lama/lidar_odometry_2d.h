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

#pragma once

#include "types.h"
#include "pose2d.h"

#include "nlls/solver.h"

#include "sdm/dynamic_distance_map.h"
#include "sdm/frequency_occupancy_map.h"
#include "sdm/probabilistic_occupancy_map.h"

namespace lama {

struct LidarOdometry2D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // internal state
    DynamicDistanceMap*    distance_map;
    ProbabilisticOccupancyMap* occupancy_map;

    Solver::Options solver_options;

    bool has_first_scan = false;

    Pose2D odom;
    Pose2D map_update_odom;


    struct Options {
        Options(){};
        /// Resolutions of the maps.
        double resolution = 0.05;
        /// Maximum number of iterations that the optimizer can achieve.
        uint32_t max_iter = 100;
    };

    LidarOdometry2D(const Options& options = Options());
    virtual ~LidarOdometry2D();

    bool update(const PointCloudXYZ::Ptr& surface, double timestamp);
    void updateMaps(const PointCloudXYZ::Ptr& surface);
};

}/* lama */
