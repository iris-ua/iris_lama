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
#include "sdm/export.h"

namespace lama {

class Slam2D {
public:

    typedef Solver::Options SolverOptions;

    typedef Strategy::Ptr   StrategyPtr;
    typedef RobustCost::Ptr RobustCostPtr;

    struct Options {
        Options();
        /// the ammount of displacement that the system must
        /// gather before any update takes place.
        double trans_thresh;

        /// the ammout of rotation that the system must
        /// gather before any update takes place.
        double rot_thresh;

        /// maximum distance (in meters) of the euclidean distance map.
        double l2_max;

        double truncated_ray;

        /// resolutions of the maps.
        double resolution;

        /// The side size of a patch
        uint32_t patch_size;

        /// maximum number of iterations that the optimizer
        /// can achieve.
        uint32_t max_iter;

        /// strategy to use in the optimization.
        std::string strategy;

        /// wether or not to keep an execution summary.
        bool keep_summary;

        /// online data compression
        bool use_compression;

        /// size of LRU
        uint32_t cache_size;

        /// compression algorithm to use when compression is activated
        std::string calgorithm;
    };


    Slam2D(const Options& options = Options());

    virtual ~Slam2D();

    bool enoughMotion(const Pose2D& odometry);

    bool update(const PointCloudXYZ::Ptr& surface, const Pose2D& odometry, double timestamp);

    uint64_t getMemoryUsage() const;
    uint64_t getMemoryUsage(uint64_t& occmem, uint64_t& dmmem) const;

    uint32_t getNumberOfProcessedCells() const
    { return number_of_proccessed_cells_; }


    void useCompression(bool compression, const std::string& algorithm = "lz4");

    inline void setPose(const Pose2D& pose)
    { pose_ = pose; }

    inline Pose2D getPose() const
    { return pose_; }

    const FrequencyOccupancyMap* getOccupancyMap() const
    { return occupancy_map_; }

    const DynamicDistanceMap* getDistanceMap() const
    { return distance_map_; }

    inline void saveOccImage(const std::string& name) const
    { sdm::export_to_png(*occupancy_map_, name); }

    inline void saveDistImage(const std::string& name) const
    { sdm::export_to_png(*distance_map_, name); }

private:

    StrategyPtr makeStrategy(const std::string& name, const VectorXd& parameters);
    RobustCostPtr makeRobust(const std::string& name, const double& param);

    void updateMaps(const PointCloudXYZ::Ptr& cloud);

private:
    SolverOptions solver_options_;

    DynamicDistanceMap*    distance_map_;
    FrequencyOccupancyMap* occupancy_map_;

    Pose2D odom_;
    Pose2D pose_;

    double trans_thresh_;
    double rot_thresh_;
    bool has_first_scan;

    uint32_t number_of_proccessed_cells_;
    double truncated_ray_;
};

} /* lama */

