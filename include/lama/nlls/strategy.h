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
#include <string>

#include "lama/types.h"

namespace lama {

struct Strategy {

    typedef std::shared_ptr<Strategy> Ptr;

    /**
     * Reset the optimization strategy to its initial state.
     */
    virtual void reset() = 0;

    /**
     * Performe an optimization step.
     *
     * @param[in]  residuals The problem residuals.
     * @param[in]  J         The Jacobian matrix.
     * @param[out] summary
     *
     * @returns The optimization step.
     */
    virtual VectorXd step(const VectorXd& residuals, const MatrixXd& J) = 0;

    /**
     * Check the validity of the optimization step.
     *
     * @param[in] residuals The residuals after the optimization step.
     *
     * @returns True if the last step is valid, false otherwise.
     */
    virtual bool valid(const VectorXd& residuals) = 0;

    /**
     * Verify if we should stop the optimization.
     */
    virtual bool stop() = 0;

    /*
     * Get the name of the strategy.
     */
    virtual std::string name() const = 0;

};

} /* lama */

