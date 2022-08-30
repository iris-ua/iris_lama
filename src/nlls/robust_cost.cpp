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

#include "lama/nlls/robust_cost.h"

double lama::UnitWeight::value(const double& )
{
    return 1.0;
}

lama::TukeyWeight::TukeyWeight(const double& b)
    : bb_(b*b)
{ }

double lama::TukeyWeight::value(const double& x)
{
    const double xx = x*x;
    if ( xx <= bb_ ){
        const double w = 1.0 - xx / bb_;

        return w*w;
    }

    return 0.0;
}

lama::TDistributionWeight::TDistributionWeight(const double& dof)
    : dof_(dof)
{}

double lama::TDistributionWeight::value(const double& x)
{
    return ((dof_ + 1.0f) / (dof_ + (x * x)));
}

lama::CauchyWeight::CauchyWeight(const double& param)
    : c_(1.0 / (param*param))
{}

double lama::CauchyWeight::value(const double& x)
{
    return (1.0 / (1.0 + x*x * c_));
}

lama::HuberWeight::HuberWeight(const double& k)
    : k_(k)
{}

double lama::HuberWeight::value(const double& x)
{
    return (x < k_) ? 1.0 : (k_ / std::fabs(x));
}

