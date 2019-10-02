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

#include <vector>
#include <string>

#include "lama/types.h"
#include "lama/nlls/problem.h"
#include "lama/nlls/strategy.h"
#include "lama/nlls/robust_cost.h"

namespace lama {

/**
 * Class for solving nonlinear least-squares (NLLS) problems.
 */
class Solver {
public:

    struct Options {
        Options();

        /// Maximum number of iterations that can be executed.
        uint32_t max_iterations;

        /// Strategy used by the solver.
        Strategy::Ptr strategy;

        /// Weight functions used by the solver.
        RobustCost::Ptr robust_cost;

        /// The solver will output the summary of each iteration,
        /// as it happens, if this flag is set to true.
        bool write_to_stdout;
    };

public:

    Solver(const Options& options = Options());

    void solve(Problem& problem, MatrixXd* cov = 0);
private:

    void computeWeights(const VectorXd& residuals, VectorXd& weights);
    void scaleJacobian(const VectorXd& weights, MatrixXd& J);
    void calculateCovariance(const MatrixXd& J, MatrixXd* cov) const;

private:
    Options options_;
};

void Solve(const Solver::Options& options, Problem& problem, MatrixXd* cov = 0);

} /* lama */

