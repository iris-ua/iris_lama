/*
 * IRIS Localization and Mapping (LaMa)
 *
 * Copyright (c) 2022-today, Eurico Pedrosa, University of Aveiro - Portugal
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

#include <algorithm>

#include <Eigen/Geometry>

#include "minisam/core/FactorGraph.h"
#include "minisam/core/LossFunction.h"
#include "minisam/geometry/Sophus.h"
#include "minisam/nonlinear/GaussNewtonOptimizer.h"
#include "minisam/nonlinear/DoglegOptimizer.h"
#include "minisam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "minisam/slam/BetweenFactor.h"
#include "minisam/slam/PriorFactor.h"
#include "minisam/utils/Timer.h"

#include "lama/match_surface_2d.h"
#include "lama/nlls/gauss_newton.h"
#include "lama/nlls/solver.h"
#include "lama/print.h"
#include "lama/timer.h"

#include "lama/sdm/dynamic_distance_map.h"
#include "lama/sdm/export.h"

#include "lama/graph_slam2d.h"

#include "nanoflann/nanoflann.hpp"

template <typename Derived> struct KeyPosesNanoFlannAdaptor {

    const Derived &obj; //!< A const ref to the data set origin
    const int ignore_n_keys;

    /// The constructor that sets the data set source
    explicit KeyPosesNanoFlannAdaptor(const Derived &obj_, int ignore_n_keys)
        : obj(obj_), ignore_n_keys(ignore_n_keys) {}

    /// CRTP helper method
    inline const Derived &derived() const { return obj; }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return derived().size() - ignore_n_keys; }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0)
            return derived()[idx].pose.x();
        else
            return derived()[idx].pose.y();
    }

    // Optional bounding-box computation: return false
    // to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the
    //   class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected
    //   dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const {
        return false;
    }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<double, KeyPosesNanoFlannAdaptor<lama::GraphSlam2D::KeyPoseList>>,
            KeyPosesNanoFlannAdaptor<lama::GraphSlam2D::KeyPoseList>, 2> KDTree;

//------------------------------------------------------

lama::GraphSlam2D::GraphSlam2D(const Options &options) : options(options) {

    this->options.transient_map = true;
    this->options.truncated_ray = 1.0; //this->options.l2_max;

    slam = std::make_shared<Slam2D>(this->options);
    graph = new sam::FactorGraph();
}

lama::GraphSlam2D::~GraphSlam2D()
{
    delete graph;
}

void lama::GraphSlam2D::Init(const Pose2D &prior)
{
    slam->setPose(prior);
}

bool lama::GraphSlam2D::enoughMotion(const Pose2D &odometry) {
    return slam->enoughMotion(odometry);
}

lama::Pose2D lama::GraphSlam2D::getPose() const {
    return (correction + slam->getPose());
}

lama::GraphSlam2D::FrequencyOccupancyMapPtr lama::GraphSlam2D::generateOccupancyMap(bool full) {
    if (mapping_keyid == 0)
        occ = std::make_shared<FrequencyOccupancyMap>(full ? options.resolution : 0.1);

    for (size_t i = mapping_keyid; i < key_poses.size(); ++i) {

        auto &node = key_poses[i];
        auto &cloud = node.cloud;
        auto &pose = node.pose;

        // 1. Transform the point cloud to the model coordinates.
        Isometry3d moving_tf = Translation3d(cloud->sensor_origin_) * cloud->sensor_orientation_;
        Isometry3d fixed_tf  = Translation3d(pose.x(), pose.y(), 0.0) * AngleAxisd(pose.rotation(), Vector3d::UnitZ());
        Isometry3d tf = fixed_tf * moving_tf;

        Vector3ui so = occ->w2m(tf.translation());

        // 2. Free positions
        for (auto &point : cloud->points) {
            Vector3d hit = tf * point;
            occ->setOccupied(hit);

            if (full)
                occ->computeRay(so, occ->w2m(hit),
                [this](const Vector3ui& coord){
                    occ->setFree(coord);
                });
        } // end for points
    } // end for

    occ->prune();
    mapping_keyid = key_poses.size();
    return occ;
}

lama::GraphSlam2D::DynamicDistanceMapPtr lama::GraphSlam2D::generateCoarseDistanceMap()
{
    auto dm = std::make_shared<DynamicDistanceMap>(0.1);
    dm->setMaxDistance(5.0);

    slam->getDistanceMap()->visit_all_cells([&](auto& coords){

        if (not slam->getOccupancyMap()->isOccupied(coords))
            return;

        auto w = slam->getDistanceMap()->m2w(coords);
        auto m = dm->w2m(w);

        dm->addObstacle(m);
    });

    dm->update();

    coarse_dm = dm;
    return dm;
}

bool lama::GraphSlam2D::update(const PointCloudXYZ::Ptr &surface, const Pose2D &odometry, double timestamp)
{
    ScopedTimer timer("GraphSlam2D.update");

    // 1. Update the transient slam
    auto did_update = slam->update(surface, odometry, timestamp);
    if (!did_update)
        return false;

    static Pose2D odom = odometry;

    // 2. Check for key pose
    static Pose2D prev(1e10, 1e10, 0.0);
    Pose2D diff = slam->getPose() - prev;

    if (diff.xy().norm() < options.key_pose_distance &&
            std::fabs(diff.rotation()) < options.key_pose_angular_distance)
        return true;

    prev = slam->getPose();

    // 4. Add key pose to pose graph and key pose list.
    int keyid = key_poses.size();

    if (keyid == 0) {

        auto loss = sam::DiagonalLoss::Sigmas(Vector3d(0.01, 0.01, 0.01));
        graph->add(sam::PriorFactor<SE2d>(sam::key('x', keyid), slam->getPose().state, loss));

    } else {

        accdist += diff.xy().norm();

        Pose2D between = key_poses.back().pose - (correction + slam->getPose());
        auto keyid = key_poses.size();
        // TODO: Use a better loss function ?
        auto loss = sam::DiagonalLoss::Sigmas(Vector3d(0.25, 0.25, 0.15));
        graph->add(sam::BetweenFactor<SE2d>(sam::key('x', keyid - 1), sam::key('x', keyid), between.state, loss));
    }

    key_poses.push_back({keyid, correction + slam->getPose(), slam->getPose(), odom - odometry, surface, timestamp});

    if (keyid < options.key_pose_head_delay ||
        keyid < options.ignore_n_chain_poses)
        return true;


    // 5. Search for loop closures
    double r = std::min(accdist, 100.0) / 100.0;
    double radius = std::pow(options.loop_search_max_distance, r) * std::pow(options.loop_search_min_distance, (1.0 - r));

    keyid -= options.key_pose_head_delay;
    Pose2D &pose = key_poses[keyid].pose;

    LoopClosureCandidates candidates = findLoopClosureCandidates(pose.xy(), radius);

    static double factordist = 0.0;
    factordist += diff.xy().norm();

    Pose2D between;
    for (size_t i = 0; i < candidates.size(); ++i){
        auto& idx = candidates[i];

        double rmse = correlateCandidateScan(keyid, idx, between);

        if (rmse > options.loop_closure_scan_rmse){
            if (i == 0){
                // one more chance but only for the closest candidate
                rmse = coarseSearchAndCorrelateCandidateScan(keyid, idx, between);
                if (rmse > options.loop_closure_scan_rmse * 2.0)
                    continue;

            } else {
                continue;
            }// end if
        }// end if

        // Add factor
        static auto loss = sam::HuberLoss::Huber(0.1);
        links.push_back(std::make_pair(idx, keyid));
        factor_queue.push(std::make_shared<sam::BetweenFactor<SE2d>>(sam::key('x', idx), sam::key('x', keyid), between.state, loss));

        factordist = 0.0;
        // Only one factor per update
        break;
    } // end for

    if (factor_queue.empty() or (factor_queue.size() <= 5 and factordist <= 15.0))
        return true;

    optimizePoseGraph();
    factordist = 0.0;

    return true;
}

lama::GraphSlam2D::LoopClosureCandidates lama::GraphSlam2D::findLoopClosureCandidates(const Vector2d &query, double radius)
{
    ScopedTimer timer("GraphSlam2D.findLoopClosureCandidates");

    KeyPosesNanoFlannAdaptor<KeyPoseList> key_poses_adaptor(key_poses, options.ignore_n_chain_poses);
    KDTree index(2, key_poses_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams{10});

    //index.buildIndex();

    KeyPosesSearchResult results;
    nanoflann::SearchParams params;

    index.radiusSearch(&(query.data()[0]), radius*radius, results, params);

    // Limit the number of candidates for lower computational complexity.
    const size_t max_candidates = options.loop_max_candidates;
    if (results.size() > max_candidates)
        results.erase(results.begin() + max_candidates, results.end());

    if (results.empty())
        return {}; // empty list

    LoopClosureCandidates lcc;
    lcc.reserve(results.size());

    for (auto &candidate : results)
        lcc.push_back(candidate.first);

    return lcc;
}

double lama::GraphSlam2D::correlateCandidateScan(int refidx, int candidate_id, Pose2D &between)
{
    ScopedTimer timer("GraphSlam2D.correlateCandidateScan");

    auto ref_pose = Pose2D(correction.state.inverse()) + key_poses[refidx].pose;
    auto candidate_pose = Pose2D(correction.state.inverse()) + key_poses[candidate_id].pose;

    /* auto &ref_cloud = key_poses[refidx].cloud; */
    auto &candidate_cloud = key_poses[candidate_id].cloud;

    Solver::Options solver_options;
    solver_options.strategy.reset(new GaussNewton{});
    solver_options.robust_cost.reset(new HuberWeight(0.15));
    solver_options.max_iterations = 1;

    MatchSurface2D match_surface0(slam->getDistanceMap(), candidate_cloud, candidate_pose.state);
    MatchSurface2D match_surface1(slam->getDistanceMap(), candidate_cloud, Pose2D(ref_pose.xy(), candidate_pose.rotation()).state);

    MatchSurface2D *msp;

    // Assume that the candidate is part of the local map.
    Solve(solver_options, match_surface0);
    double rmse0 = match_surface0.error();

    // Assume that the candidate is not part of the local map.
    Solve(solver_options, match_surface1);
    double rmse1 = match_surface1.error();

    // use the one with the lowest error
    if (rmse0 < rmse1)
        msp = &match_surface0;
    else
        msp = &match_surface1;

    // 2.0 Finish the optimization.
    solver_options.max_iterations = 100;
    Solve(solver_options, *msp);

    between = Pose2D(msp->getState()) - ref_pose;
    return msp->error();
}

double lama::GraphSlam2D::coarseSearchAndCorrelateCandidateScan(int refidx, int candidate_id, Pose2D& between)
{
    ScopedTimer timer("GraphSlam2D.coarseSearchAndCorrelateCandidateScan");

    auto ref_pose = Pose2D(correction.state.inverse()) + key_poses[refidx].pose;
    auto candidate_pose = Pose2D(correction.state.inverse()) + key_poses[candidate_id].pose;

    auto &ref_cloud = key_poses[refidx].cloud;
    auto &candidate_cloud = key_poses[candidate_id].cloud;

    Solver::Options solver_options;
    solver_options.strategy.reset(new GaussNewton{});
    solver_options.robust_cost.reset(new HuberWeight(0.15));

    Affine3d moving_tf = Translation3d(ref_cloud->sensor_origin_) * ref_cloud->sensor_orientation_;
    Vector3d trans; trans << ref_pose.x(),
             ref_pose.y(),
             0.0;
    Affine3d fixed_tf = Translation3d(trans) * AngleAxisd(ref_pose.state.so2().log(), Vector3d::UnitZ());
    Affine3d tf = fixed_tf * moving_tf;

    DynamicDistanceMapPtr dm = std::make_shared<DynamicDistanceMap>(0.25);
    dm->setMaxDistance(2.5);
    for (auto& point : ref_cloud->points)
        dm->addObstacle(dm->w2m(tf*point));
    dm->update();

    MatchSurface2D match_surface(dm.get(), candidate_cloud, candidate_pose.state);
    Solve(solver_options, match_surface);

    match_surface.surface_ = slam->getDistanceMap();
    Solve(solver_options, match_surface);

    between = Pose2D(match_surface.getState()) - ref_pose;
    return match_surface.error();
}

void lama::GraphSlam2D::optimizePoseGraph()
{
    if (factor_queue.empty())
        return;

    while (!factor_queue.empty()) {
        graph->add(factor_queue.front());
        factor_queue.pop();
    }

    sam::LevenbergMarquardtOptimizerParams opt_param;
    opt_param.verbosity_level = sam::NonlinearOptimizerVerbosityLevel::NONE;
    sam::LevenbergMarquardtOptimizer opt(opt_param);

    sam::Variables result;
    sam::Variables initial;
    for (size_t i = 0; i < key_poses.size(); ++i)
        initial.add(sam::key('x', i), key_poses[i].pose.state);

    auto status = opt.optimize(*graph, initial, result);
    if (status == sam::NonlinearOptimizationStatus::SUCCESS) {
        // propagate corrections
        for (auto &kv : result) {
            auto idx = sam::keyIndex(kv.first);
            key_poses[idx].pose.state = kv.second->cast<SE2d>();
        }

        auto A = key_poses.back().pose.state;
        auto B = slam->getPose().state;
        auto C = B * A.inverse();

        correction = Pose2D(C.inverse());
    }

    mapping_keyid = 0;
    accdist = 0.0;
}


