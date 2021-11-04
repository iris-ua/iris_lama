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

#include "types.h"
#include "pose2d.h"

#include "nlls/solver.h"

#include "sdm/dynamic_distance_map.h"
#include "sdm/simple_occupancy_map.h"

namespace lama {

class Loc2D {
public:

    typedef Solver::Options SolverOptions;

    typedef Strategy::Ptr   StrategyPtr;
    typedef RobustCost::Ptr RobustCostPtr;

    typedef std::shared_ptr<DynamicDistanceMap>    DynamicDistanceMapPtr;
    typedef std::shared_ptr<SimpleOccupancyMap>    SimpleOccupancyMapPtr;

public:

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

        /// resolutions of the maps.
        double resolution;

        /// The side size of a patch
        uint32_t patch_size;

        /// maximum number of iterations that the optimizer
        /// can achieve.
        uint32_t max_iter;

        /// strategy to use in the optimization.
        std::string strategy;

        /// number of particles used for global localization.
        uint32_t gloc_particles;

        /// maximum number of global localization iterations.
        uint32_t gloc_iters;

        /// RMSE threshold to accept the global localization estimate.
        double gloc_thresh;

        /// Blending factor between the covariance from the optimization
        /// and the sampling covariance.
        double cov_blend;
    };

public:

    SimpleOccupancyMap* occupancy_map;
    DynamicDistanceMap* distance_map;

    Loc2D() = default;
    void Init(const Options& options = Options());

    virtual ~Loc2D();

    bool enoughMotion(const Pose2D& odometry);

    bool update(const PointCloudXYZ::Ptr& surface, const Pose2D& odometry, double timestamp, bool force_update = false);

    void triggerGlobalLocalization();

    inline void setPose(const Pose2D& pose)
    { pose_ = pose; has_first_scan = false; }

    inline const Pose2D& getPose() const
    { return pose_; }

    inline const Matrix3d& getCovar() const
    { return cov_; }

    inline double getRMSE() const
    { return rmse_; }

    inline bool globalLocalizationIsActive() const
    { return do_global_localization_; }

private:

    void addSamplingCovariance(const PointCloudXYZ::Ptr& surface);

    void globalLocalization(const PointCloudXYZ::Ptr& surface);

    StrategyPtr makeStrategy(const std::string& name, const VectorXd& parameters);
    RobustCostPtr makeRobust(const std::string& name, const double& param);

private:
    SolverOptions solver_options_;

    Pose2D odom_;
    Pose2D pose_;
    Matrix3d cov_;

    VectorVector2d sampling_steps_;
    double cov_blend_;

    double trans_thresh_;
    double rot_thresh_;
    double rmse_;

    bool has_first_scan;
    bool do_global_localization_;

    double gloc_particles_;
    double gloc_thresh_;
    uint32_t gloc_iters_;
    uint32_t gloc_cur_iter_;

};

} /* lama */

