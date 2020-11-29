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

#include <memory>
#include <vector>

#include "time.h"
#include "pose2d.h"
#include "nlls/solver.h"

#include "sdm/dynamic_distance_map.h"
#include "sdm/frequency_occupancy_map.h"

#include <Eigen/StdVector>

namespace lama {

struct ThreadPool;

class PFSlam2D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Solver::Options SolverOptions;

    typedef Strategy::Ptr   StrategyPtr;
    typedef RobustCost::Ptr RobustCostPtr;

    typedef std::shared_ptr<DynamicDistanceMap>    DynamicDistanceMapPtr;
    typedef std::shared_ptr<FrequencyOccupancyMap> FrequencyOccupancyMapPtr;


public:

    struct Particle {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // The weight of the particle
        double weight;

        double normalized_weight;

        double weight_sum;

        // The pose of this particle in the map
        Pose2D pose;

        // history
        DynamicArray<Pose2D> poses;

        DynamicDistanceMapPtr    dm;
        FrequencyOccupancyMapPtr occ;

    };

    struct Summary {
        /// When it happend.
        DynamicArray<double> timestamp;
        /// Total execution time.
        DynamicArray<double> time;
        /// Solving (i.e. optimization) execution time.
        DynamicArray<double> time_solving;
        /// Normalizing execution time.
        DynamicArray<double> time_normalizing;
        /// Resampling execution time.
        DynamicArray<double> time_resampling;
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

    inline void probeNTime(Duration elapsed)
    { summary->time_normalizing.push_back(elapsed.toSec()); }

    inline void probeRTime(Duration elapsed)
    { summary->time_resampling.push_back(elapsed.toSec()); }

    inline void probeMTime(Duration elapsed)
    { summary->time_mapping.push_back(elapsed.toSec()); }

    inline void probeMem()
    { summary->memory.push_back(getMemoryUsage()); }

    // All SLAM options are in one place for easy access and passing.
    struct Options {
        Options(){}

        /// The number of particles to use
        uint32_t particles;
        /// How much the rotation affects rotation.
        double srr = 0.1;
        /// How much the translation affects rotation.
        double str = 0.2;
        /// How much the translation affects translation.
        double stt = 0.1;
        /// How much the rotation affects translation.
        double srt = 0.2;
        /// Measurement confidence.
        double meas_sigma = 0.05;
        /// Use this to smooth the measurements likelihood.
        double meas_sigma_gain = 3;
        /// The ammount of displacement that the system must
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
        /// The side size of a patch
        uint32_t patch_size = 32;
        /// Maximum number of iterations that the optimizer
        /// can achieve.
        uint32_t max_iter = 100;
        /// Strategy to use in the optimization.
        std::string strategy = "gn";
        /// Number of working threads.
        /// -1 for none, 0 for auto, >0 user define number of workers.
        int32_t threads = -1;
        /// Pseudo random generator seed.
        /// Use 0 to generate a new seed.
        uint32_t seed = 0;
        /// Should online data compression be used or not.
        bool use_compression = false;
        /// Size of LRU.
        uint32_t cache_size = 100;
        /// Compression algorithm to use when compression is activated
        std::string calgorithm = "lz4";
        /// Save data to create an execution summary.
        bool create_summary = false;
    };

    PFSlam2D(const Options& options = Options());

    virtual ~PFSlam2D();

    inline const Options& getOptions() const
    {
        return options_;
    }

    uint64_t getMemoryUsage() const;
    uint64_t getMemoryUsage(uint64_t& occmem, uint64_t& dmmem) const;

    bool update(const PointCloudXYZ::Ptr& surface, const Pose2D& odometry, double timestamp);

    size_t getBestParticleIdx() const;

    Pose2D getPose() const;

    inline const std::deque<double>& getTimestamps() const
    { return timestamps_; }

    inline const std::vector<Particle>& getParticles() const
    { return particles_[current_particle_set_]; }

    const FrequencyOccupancyMap* getOccupancyMap() const
    {
        if (!has_first_scan) return nullptr;

        size_t pidx = getBestParticleIdx();
        return particles_[current_particle_set_][pidx].occ.get();
    }

    const DynamicDistanceMap* getDistanceMap() const
    {
        if (!has_first_scan) return nullptr;

        size_t pidx = getBestParticleIdx();
        return particles_[current_particle_set_][pidx].dm.get();
    }

    void saveOccImage(const std::string& name) const;

    inline double getNeff() const
    { return neff_; }

    void setPrior(const Pose2D& prior);

private:

    StrategyPtr makeStrategy(const std::string& name, const VectorXd& parameters);
    RobustCostPtr makeRobust(const std::string& name, const double& param);

    void drawFromMotion(const Pose2D& delta, const Pose2D& old_pose, Pose2D& pose);

    double likelihood(const PointCloudXYZ::Ptr& surface, Pose2D& pose);

    double calculateLikelihood(const PointCloudXYZ::Ptr& surface, const Pose2D& pose);
    double calculateLikelihood(const Particle& particle);

    void scanMatch(Particle* particle);
    void updateParticleMaps(Particle* particle);

    void normalize();
    void resample();

private:
    Options options_;
    SolverOptions solver_options_;

    std::vector<Particle> particles_[2];
    uint8_t current_particle_set_;

    Pose2D odom_;
    Pose2D pose_;

    double acc_trans_;
    double acc_rot_;
    bool   has_first_scan;

    double truncated_ray_;
    double truncated_range_;
    double max_weight_;
    double delta_free_;
    double neff_;

    std::deque<double> timestamps_;
    PointCloudXYZ::Ptr current_surface_;

    ThreadPool* thread_pool_;
};

} /* lama */

