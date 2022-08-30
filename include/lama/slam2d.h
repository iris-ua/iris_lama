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
#include "time.h"
#include "pose2d.h"

#include "nlls/solver.h"

#include "sdm/dynamic_distance_map.h"
#include "sdm/frequency_occupancy_map.h"
#include "sdm/export.h"

namespace lama {

class Slam2D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Solver::Options SolverOptions;

    typedef Strategy::Ptr   StrategyPtr;
    typedef RobustCost::Ptr RobustCostPtr;

    struct Summary {
        /// When it happend.
        DynamicArray<double> timestamp;
        /// Total execution time.
        DynamicArray<double> time;
        /// Solving (i.e. optimization) execution time.
        DynamicArray<double> time_solving;
        /// Mapping (occ+distance) execution time.
        DynamicArray<double> time_mapping;

        /// Total memory used by the maps.
        DynamicArray<double> memory;

        std::string report() const;
    };
    Summary* summary = nullptr;

    // Summary help functions.
    inline void probeStamp(double stamp)
    { summary->timestamp.push_back(stamp); }

    inline void probeTime(Duration elapsed)
    { summary->time.push_back(elapsed.toSec()); }

    inline void probeSTime(Duration elapsed)
    { summary->time_solving.push_back(elapsed.toSec()); }

    inline void probeMTime(Duration elapsed)
    { summary->time_mapping.push_back(elapsed.toSec()); }

    inline void probeMem()
    { summary->memory.push_back(getMemoryUsage()); }

    // All SLAM options are in one place for easy access and passing.
    struct Options {
        Options(){}

        /// The ammount of linear motion that the system must
        /// gather before any update takes place.
        double trans_thresh = 0.5;
        /// The ammout of rotation that the system must
        /// gather before any update takes place.
        double rot_thresh = 0.5;
        /// Maximum distance (in meters) of the euclidean distance map.
        double l2_max = 0.5;
        /// If != from zero, truncate the ray lenght (includes the endpoint).
        double truncated_ray = 0.0;
        /// If != from zero and ray length > truncated_range, truncate the ray from the
        /// starting point and do not add an obstacle for the hit
        double truncated_range = 0.0;
        /// Resolutions of the maps.
        double resolution = 0.05;
        /// The side size of a patch.
        uint32_t patch_size = 32;
        /// Maximum number of iterations that the optimizer can achieve.
        uint32_t max_iter = 100;
        /// Strategy to use in the optimization.
        std::string strategy = "gn";
        /// Should online data compression be used or not.
        bool use_compression = false;
        /// Size of LRU.
        uint32_t cache_size = 100;
        /// Compression algorithm to use when compression is activated
        std::string calgorithm = "lz4";
        // Only keep the most recent area of the map.
        bool transient_map = false;
        /// Save data to create an execution summary.
        bool create_summary = false;
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

    StrategyPtr makeStrategy(const std::string& name);
    RobustCostPtr makeRobust(const std::string& name);

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
    double truncated_range_;

    bool transient_map_;
};

} /* lama */

