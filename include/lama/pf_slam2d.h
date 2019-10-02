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

#include "pose2d.h"

#include "nlls/solver.h"

#include "sdm/dynamic_distance_map.h"
#include "sdm/frequency_occupancy_map.h"

namespace lama {

struct ThreadPool;

class PFSlam2D {
public:

    typedef Solver::Options SolverOptions;

    typedef Strategy::Ptr   StrategyPtr;
    typedef RobustCost::Ptr RobustCostPtr;

    typedef std::shared_ptr<DynamicDistanceMap>    DynamicDistanceMapPtr;
    typedef std::shared_ptr<FrequencyOccupancyMap> FrequencyOccupancyMapPtr;


public:

    struct Particle {

        // The weight of the particle
        double weight;

        double normalized_weight;

        double weight_sum;

        // The pose of this particle in the map
        Pose2D pose;

        // history
        std::deque<Pose2D> poses;

        DynamicDistanceMapPtr    dm;
        FrequencyOccupancyMapPtr occ;

    };

    struct Options {
        Options();

        /// the number of particles to use
        uint32_t particles;

        ///
        bool use_gaussian_proposal;

        double srr;
        double str;
        double stt;
        double srt;

        double meas_sigma;
        double meas_sigma_gain;

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

        int32_t threads;

        uint32_t seed;

        ///
        bool use_compression;
        uint32_t cache_size;

        /// compression algorithm to use when compression is activated
        std::string calgorithm;

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
        size_t pidx = getBestParticleIdx();
        return particles_[current_particle_set_][pidx].occ.get();
    }

    const DynamicDistanceMap* getDistanceMap() const
    {
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
    double max_weight_;
    double delta_free_;
    double neff_;

    std::deque<double> timestamps_;
    PointCloudXYZ::Ptr current_surface_;

    ThreadPool* thread_pool_;
};

} /* lama */

