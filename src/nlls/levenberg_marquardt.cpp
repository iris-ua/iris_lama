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

#include <cmath>
#include <Eigen/Cholesky>

#include "lama/nlls/levenberg_marquardt.h"

lama::LevenbergMarquard::Options::Options()
{
    eps1 = 1e-4;
    eps2 = 1e-4;
    tau  = 1e-4;
}

lama::LevenbergMarquard::LevenbergMarquard(const Options& options)
    : opt_(options)
{}

void lama::LevenbergMarquard::reset()
{
    mu_ = -1;
    v_  = 2.0;
    stop_ = false;
}

Eigen::VectorXd lama::LevenbergMarquard::step(const VectorXd& residuals, const MatrixXd& J)
{
    chi2_ = residuals.squaredNorm();
    g_    = J.transpose() * residuals;

    double max_abs_g = g_.lpNorm<Eigen::Infinity>();
    if (max_abs_g < opt_.eps1){
        stop_ = true;
        return VectorXd::Zero(J.cols());
    }

    MatrixXd A = J.transpose() * J;
    // mu_ is allways positive because the diagonal of A is also allways
    // positive. Therefore, a negative mu_ requires initialization.
    if ( mu_ < 0 )
        mu_ = opt_.tau * A.diagonal().maxCoeff();

    A.diagonal().array() += (mu_ );
    // Solve the system
    h_ = A.selfadjointView<Eigen::Upper>().llt().solve(-g_);

    double max_abs_h = h_.lpNorm<Eigen::Infinity>();
    if (max_abs_h < opt_.eps2)
        stop_ = true;

    return h_;
}

bool lama::LevenbergMarquard::valid(const VectorXd& residuals)
{
    if (stop_) return true;

    double dF = (chi2_ - residuals.squaredNorm());
    double dL = 0.5*h_.transpose().dot(mu_*h_ - g_);

    if ( dL > 0.0 and dF > 0.0 ){
        // update mu
        mu_ = mu_ * std::max( 1.0 / 3.0, 1 - std::pow(2*(dF/dL)-1,3) );
        v_  = 2.0;

        return true;
    }

    mu_ = mu_ * v_; v_ = 2*v_;
    return false;
}

bool lama::LevenbergMarquard::stop()
{
    return stop_;
}

