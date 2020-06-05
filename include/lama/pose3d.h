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

namespace lama {

struct Pose3D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose3D();
    Pose3D(const double& x, const double& y, const double& z,
           const double& roll, const double& pitch, const double& yaw);
    Pose3D(const Vector3d& xyz, const Vector3d& rpy);
    Pose3D(const Vector3d& xyz, const double& yaw);
    Pose3D(const Matrix4d& transformation);
    Pose3D(const Affine3d& transformation);
    Pose3D(const Pose3D& other);
    Pose3D(const SE3d& se3);

    virtual ~Pose3D();

    Pose3D operator+(const Pose3D& other);
    Pose3D operator-(const Pose3D& other);

    Vector3d operator*(const Vector3d& point);

    Pose3D& operator+=(const Pose3D& other);
    Pose3D& operator-=(const Pose3D& other);

    double x() const;
    double y() const;
    double z() const;
    Vector3d xyz() const;

    double roll() const;
    double pitch() const;
    double yaw() const;
    Vector3d rpy() const;

    SE3d state;
};

} /* lama */

