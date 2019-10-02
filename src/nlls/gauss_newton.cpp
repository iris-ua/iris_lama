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

#include <Eigen/Cholesky>

#include "lama/nlls/gauss_newton.h"

lama::GaussNewton::Options::Options()
{
    eps1 = 1e-4;
    eps2 = 1e-4;
}

lama::GaussNewton::GaussNewton(const Options& options)
    : opt_(options)
{}

void lama::GaussNewton::reset()
{
    stop_ = false;
}

Eigen::VectorXd lama::GaussNewton::step(const VectorXd& residuals, const MatrixXd& J)
{
    VectorXd g = J.transpose() * residuals;
    chi2_ = residuals.squaredNorm();

    double max_abs_g = g.lpNorm<Eigen::Infinity>();
    if (max_abs_g < opt_.eps1){
        stop_ = true;
        return VectorXd::Zero(J.cols());
    }

    MatrixXd A = J.transpose() * J;
    // Solve the system
    VectorXd h = A.selfadjointView<Eigen::Lower>().ldlt().solve(-g);

    double max_abs_h = h.lpNorm<Eigen::Infinity>();
    if (max_abs_h < opt_.eps2)
        stop_ = true;

    return h;
}

bool lama::GaussNewton::valid(const VectorXd& residuals)
{
    if (stop_) return true;

    double dF  = chi2_ - residuals.squaredNorm();
    if (dF > 0){
        return true;
    }

    stop_ = true;
    return false;
}

bool lama::GaussNewton::stop()
{
    return stop_;
}

