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

#pragma once

#include "types.h"
#include "pose2d.h"

#include "slam2d.h"

// forward declarations
namespace minisam {
class FactorGraph;
class Factor;
}

namespace sam = minisam;

namespace lama {

class GraphSlam2D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using FrequencyOccupancyMapPtr = std::shared_ptr<FrequencyOccupancyMap>;
    using DynamicDistanceMapPtr    = std::shared_ptr<DynamicDistanceMap>;

    // All SLAM options are in one place for easy access and passing.
    struct Options : public Slam2D::Options {
        Options() : Slam2D::Options(){}

        // Linear distance between key poses.
        double key_pose_distance = 1.0;
        double key_pose_angular_distance = 0.5 * M_PI;

        // Head delay for loop closure reference.
        int key_pose_head_delay = 5;

        // Maximum search radius for loop closure;
        double loop_search_max_distance = 10.0;
        // Minimum search radius for loop closure;
        double loop_search_min_distance = 2.0;

        // Maximum number of loop candidates
        int loop_max_candidates = 5;

        // Maximum rmse value a scan match must have to be
        // considered a valid loop closure
        double loop_closure_scan_rmse = 0.05;

        // Maximum number of candidates considered for loop closure.
        int loop_closure_max_candidates = 10;

        // Ignore this amount of poses that belong to the
        // same chain for loop closure evaluation.
        int ignore_n_chain_poses = 20;
    };

    // Keep a copy for our selfs.
    Options options;

    // A key poses is a pose in the robot's path that is considered
    // for pose graph optimization. Each key-pose contains and unique id,
    // the global pose (which will be optimized) and the corresponding
    // laser scan as a pointcloud. The laser scan will be used by the loop
    // closure process to create associations.
    struct KeyPose {
        int id;
        Pose2D pose;
        Pose2D original;
        Pose2D odom;
        PointCloudXYZ::Ptr cloud;
        double timestamp;
    };

    std::queue<KeyPose> key_pose_buffer;

    // The list with all the key-poses.
    using KeyPoseList = List<KeyPose>;
    KeyPoseList key_poses;

    double accdist = 0.0;

    // When the loop closure process finds an association it creates
    // a link between two key-poses. Each link will result in a factor.
    using LinkList = List<std::pair<int,int>>;
    LinkList links;

    // The loop closure process will do a radius search to find
    // a list of key-poses for a possible association.
    // This structure holds the radius search results, it contains a list
    // of key-poses ids and their corresponding distance to the reference key-pose.
    using KeyPosesSearchResult = std::vector<std::pair<uint32_t, double>>;

    using FactorQueue = std::queue<std::shared_ptr<sam::Factor>>;
    FactorQueue factor_queue;

    sam::FactorGraph* graph = nullptr;

    int mapping_keyid = 0;
    FrequencyOccupancyMapPtr occ;
    DynamicDistanceMapPtr coarse_dm;

    using Slam2DPtr = std::shared_ptr<Slam2D>;
    Slam2DPtr slam;

    Pose2D correction;

  public: // Functions

    GraphSlam2D(const Options& options = Options());

    virtual ~GraphSlam2D();

    // Initialize the SLAM processor with the given prior.
    void Init(const Pose2D& prior);

    bool enoughMotion(const Pose2D& odometry);

    bool update(const PointCloudXYZ::Ptr& surface, const Pose2D& odometry, double timestamp);

    Pose2D getPose() const;

    FrequencyOccupancyMapPtr generateOccupancyMap(bool full = false);

    DynamicDistanceMapPtr generateCoarseDistanceMap();

    // Candidates for loop closure is a list of key poses ids.
    using LoopClosureCandidates = std::vector<int>;

    // Find all possible key poses for loop closure within the given search radius.
    // It returs the poses grouped by chain ordered by non-ascending id.
    // Each chain is orderd by ascending distance to the query point (i.e. the latest key pose position).
    LoopClosureCandidates findLoopClosureCandidates(const Vector2d& query, double radius);

    /* double correlateCandidateScan(const Pose2D& candidate_pose, const PointCloudXYZ::Ptr& candidate_cloud, Pose2D& between); */
    double correlateCandidateScan(int refidx, int idx, Pose2D& between);

    double coarseSearchAndCorrelateCandidateScan(int refidx, int candidate_id, Pose2D& between);

    void optimizePoseGraph();

};

} /* lama */

