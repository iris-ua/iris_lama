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

#include <Eigen/Geometry>

#include "types.h"
#include "lie.h"

namespace lama  {

struct Pose2D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose2D();
    Pose2D(const double& x, const double& y, const double& rotation);
    Pose2D(const Vector2d& xy, const double& rotation);
    Pose2D(const Vector3d& xyr);
    Pose2D(const Affine2d& transformation);
    Pose2D(const Pose2D& other);
    Pose2D(const SE2d& se2);

    virtual ~Pose2D();

    Pose2D operator+(const Pose2D& other);
    Pose2D operator-(const Pose2D& other);

    Vector2d operator*(const Vector2d& point);

    Pose2D& operator+=(const Pose2D& other);
    Pose2D& operator-=(const Pose2D& other);

    Pose2D& operator=(const Pose2D& other);

    double x() const;
    double y() const;
    Vector2d xy() const;

    double rotation() const;

    const Vector3d xyr() const;

    // The pose is represented by a special euclidean group.
    SE2d state;
};

} /* lama */

