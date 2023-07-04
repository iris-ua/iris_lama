/*
 * IRIS Localization and Mapping (LaMa)
 *
 * Copyright (c) 2023-today, Eurico Pedrosa, University of Aveiro - Portugal
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

#include <minisam/core/FactorGraph.h>
#include <minisam/core/LossFunction.h>

#include <minisam/geometry/Sophus.h>

#include <minisam/slam/BetweenFactor.h>
#include <minisam/slam/PriorFactor.h>

#include <minisam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "lama/simple_pgo.h"

namespace sam = minisam;

bool lama::SimplePGO::optimize()
{
    sam::FactorGraph graph;

    if (fixed_list.empty()){
        // Keep the first pose fixed if none other fixed node exist
        const auto prior_loss = sam::DiagonalLoss::Sigmas(Vector3d(1.0,1.0,1.0));
        graph.add(sam::PriorFactor<SE2d>(sam::key('x', 0), node_list[0].state, prior_loss));
    } else {

        static const auto prior_loss = sam::DiagonalLoss::Sigmas(Vector3d(0.1,0.1,0.1));
        for (auto& node : fixed_list){
            graph.add(sam::PriorFactor<SE2d>(sam::key('x', node.first), node.second.state , prior_loss));
        }
    }

    // odometry measurement loss function
    static auto odom_loss = sam::DiagonalLoss::Sigmas(Vector3d(0.5, 0.5, 0.1));

    // add odometry factors
    const size_t num_nodes = node_list.size();
    for (size_t i = 0; i < num_nodes - 1; ++i){
        auto diff = node_list[i] - node_list[i+1];
        graph.add(sam::BetweenFactor<SE2d>(sam::key('x', i), sam::key('x', i+1), diff.state, odom_loss));
    }

    // loop closure measurement loss function
    const auto loop_loss = sam::DiagonalLoss::Sigmas(Vector3d(0.5, 0.5, 0.1));
    for (auto& pair : edge_list){
        auto from = pair.first;
        auto to   = pair.second.first;

        graph.add(sam::BetweenFactor<SE2d>(sam::key('x', from), sam::key('x', to), pair.second.second.state, loop_loss));
    }

    // Variables to optimize
    sam::Variables initial;
    for (size_t i = 0; i < num_nodes; ++i)
        initial.add(sam::key('x', i), node_list[i].state);

    // Optimize
    sam::LevenbergMarquardtOptimizerParams opt_param;
    opt_param.verbosity_level = sam::NonlinearOptimizerVerbosityLevel::ITERATION;
    sam::LevenbergMarquardtOptimizer opt(opt_param);

    sam::Variables result;
    auto status = opt.optimize(graph, initial, result);

    if (status != sam::NonlinearOptimizationStatus::SUCCESS)
        return false;

    for(auto& kv : result) {
        auto idx = sam::keyIndex(kv.first);
        node_list[idx].state = kv.second->cast<SE2d>();
    }

    return true;
}
