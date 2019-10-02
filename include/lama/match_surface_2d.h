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

#include "lie.h"
#include "nlls/problem.h"

#include "sdm/dynamic_distance_map.h"

#include <vector>

namespace lama {

struct MatchSurface2D : public Problem {

    MatchSurface2D(const DynamicDistanceMap*  surface,
                   const PointCloudXYZ::Ptr& scan,
                   const SE2d& estimate);

    /**
     *
     */
    inline SE2d getState() const
    { return state_; }

    /**
     *  Compute residuals and Jacobian.
     *  The Jacobian is only calculated if J is not null.
     *
     *  @param[out] residuals Computed residuals.
     *  @param[out] J         Jacobian matrix.
     */
    void eval(VectorXd& residuals, MatrixXd* J);

    /**
     * Update the internal state.
     *
     * @param[in] h Optimization step.
     */
    void update(const VectorXd& h);

    const DynamicDistanceMap* surface_;
    PointCloudXYZ::Ptr      scan_;

    SE2d state_;
};

} /* lama */

