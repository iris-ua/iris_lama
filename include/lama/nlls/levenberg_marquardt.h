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

#include "lama/nlls/strategy.h"

namespace lama {

struct LevenbergMarquard : public Strategy {

    struct Options {
        Options();
        /// Threshold of the first termination criterion
        /// An optimization step will terminate when max(abs( J'r )) < eps1
        double eps1;

        /// Threshold of the second termination criterion
        /// An optimization step will terminate when max(abs( h )) < eps2,
        //  where h is the solution for J'J*h = J'r.
        double eps2;

        /// Coefficient used in the computation of the inital trust radius.
        double tau;
    };

    LevenbergMarquard(const Options& options = Options());

    /**
     * Reset the optimization strategy to its initial state.
     */
    void reset();

    /**
     * Performe an optimization step.
     *
     * @param[in]  residuals The problem residuals.
     * @param[in]  J         The Jacobian matrix.
     * @param[out] summary
     *
     * @returns The optimization step.
     */
    VectorXd step(const VectorXd& residuals, const MatrixXd& J);

    /**
     * Check the validity of the optimization step.
     *
     * @param[in] residuals The residuals after the optimization step.
     *
     * @returns True if the last step is valid, false otherwise.
     */
    bool valid(const VectorXd& residuals);

    /**
     * Verify if we should stop the optimization.
     *
     */
    bool stop();

    /*
     * Get the name of the strategy.
     */
    inline std::string name() const
    { return "Levenberg-Marquard"; }

    Options opt_;
    double mu_;
    double v_;

    VectorXd h_;
    VectorXd g_;

    double chi2_;
    bool   stop_;
};

} /* lama */

