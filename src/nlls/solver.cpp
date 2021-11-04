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

#include "lama/print.h"

#include "lama/nlls/solver.h"
#include "lama/nlls/gauss_newton.h"

lama::Solver::Options::Options()
{
    max_iterations = 100;

    strategy.reset(new GaussNewton);
    robust_cost.reset(new UnitWeight);

    write_to_stdout = false;
}

lama::Solver::Solver(const Solver::Options& options)
    : options_(options)
{}

void lama::Solver::solve(Problem& problem, MatrixXd* cov)
{
    VectorXd r;  // Current residuals.
    VectorXd ur; // Updated residuals residuals.
    MatrixXd J;  // Jacobian with respect to the parameters.
    VectorXd h;  // current step

    VectorXd w;

    Strategy::Ptr strategy = options_.strategy;

    strategy->reset();
    bool valid = true;
    uint32_t iter = 0;
    while (not strategy->stop() and iter < options_.max_iterations){

        if (valid){
            // 1. compute residuals and jacobian
            problem.eval(r, &J);

            // 2. compute and apply weights
            const int32_t rows = r.rows();
            for (int32_t i = 0; i < rows; i++){
                double w = std::sqrt(options_.robust_cost->value(r[i]));
                r[i]    *= w;
                J.row(i) *= w;
            }
        }

        // 3. do optimization step
        h = strategy->step(r, J);
        if (strategy->stop())
            // convergence has been reached.
            break;

        // 4. update and verify step validity
        problem.update(h);
        problem.eval(ur, 0);

        const int32_t rows = r.rows();
        for (int32_t i = 0; i < rows; i++){
            double w = std::sqrt(options_.robust_cost->value(ur[i]));
            ur[i]    *= w;
        }

        valid = strategy->valid(ur);
        if (not valid){
            // revert the step
            problem.update(-h);
        }

        // 5. increment iteration count
        ++iter;

    }

    if (cov){
        problem.eval(r, &J);
        w.resize(r.size());
        computeWeights(r, w);
        scaleJacobian(w, J);

        calculateCovariance(J, cov);
    }
}

void lama::Solver::computeWeights(const VectorXd& residuals, VectorXd& weights)
{
    const int32_t rows = residuals.rows();
    for (int32_t i = 0; i < rows; i++)
        weights[i] = std::sqrt(options_.robust_cost->value(residuals[i]));
}

void lama::Solver::scaleJacobian(const VectorXd& weights, MatrixXd& J)
{
    const int32_t rows = weights.rows();
    for (int32_t i = 0; i < rows; ++i)
        J.row(i) *= weights[i];
}

void lama::Solver::calculateCovariance(const MatrixXd& J, MatrixXd* cov) const
{
    Eigen::ColPivHouseholderQR<MatrixXd> qr(J);

    bool QRok = (qr.info() == Eigen::Success) &&
                (qr.rank() == J.cols());

    if ( QRok ){
        *cov = (J.transpose() * J).inverse();
    }else{
        Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinV);
        Eigen::VectorXd sv = svd.singularValues();

        const double eps=1.e-3; // choose your tolerance wisely!
        sv = (sv.array().abs() > eps).select(sv.array().square().inverse(), 3.0);
        *cov = (svd.matrixV() * sv.asDiagonal() * svd.matrixV().transpose());
    }
}

//==================================================================================================

void lama::Solve(const Solver::Options& options, Problem& problem, MatrixXd* cov)
{
    Solver solver(options);
    solver.solve(problem, cov);
}

