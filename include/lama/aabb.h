/*
 * IRIS Localization and Mapping (LaMa)
 *
 * Copyright (c) 2021-today, Eurico Pedrosa, University of Aveiro - Portugal
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

namespace lama {

// Angle Axis Bounding Box.
struct AABB {
    // The center point of the AABB
    Vector3d center;
    // The half width of the AABB
    Vector3d hwidth;

    // default constructor
    AABB() = default;

    // Create an AABB given the center and the half width.
    AABB(const Vector3d& c, const Vector3d& hw)
        : center(c), hwidth(hw)
    {}

    // Create an AABB given the lower and the upper coordinates.
    AABB(const Vector3ui& min, const Vector3ui& max)
    {
        Vector3d local = (max.cast<double>() - min.cast<double>());
        hwidth = local * 0.5;
        center = min.cast<double>() + hwidth;
    }

    inline bool testIntersection(const AABB& other) const
    {
        bool x = std::abs(center(0) - other.center(0)) <= (hwidth(0) + other.hwidth(0));
        bool y = std::abs(center(1) - other.center(1)) <= (hwidth(1) + other.hwidth(1));
        bool z = std::abs(center(2) - other.center(2)) <= (hwidth(2) + other.hwidth(2));

        return (x && y && z);
    }

};

}

