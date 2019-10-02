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

#include "lama/pose3d.h"

lama::Pose3D::Pose3D()
    : state()
{}

lama::Pose3D::Pose3D(const double& x, const double& y, const double& z,
               const double& roll, const double& pitch, const double& yaw)
{
    Quaterniond tmp = AngleAxisd(roll, Vector3d::UnitX()) *
                      AngleAxisd(pitch, Vector3d::UnitY()) *
                      AngleAxisd(yaw, Vector3d::UnitZ());

    state.translation() = Vector3d(x,y,z);
    state.setQuaternion(tmp);
}

lama::Pose3D::Pose3D(const Vector3d& xyz, const double& yaw)
{
    Quaterniond tmp = AngleAxisd(0, Vector3d::UnitX()) *
                      AngleAxisd(0, Vector3d::UnitY()) *
                      AngleAxisd(yaw, Vector3d::UnitZ());

    state.translation() = xyz;
    state.setQuaternion(tmp);
}

lama::Pose3D::Pose3D(const Vector3d& xyz, const Vector3d& rpy)
{
    Quaterniond tmp = AngleAxisd(rpy(0), Vector3d::UnitX()) *
                      AngleAxisd(rpy(1), Vector3d::UnitY()) *
                      AngleAxisd(rpy(2), Vector3d::UnitZ());

    state.translation() = xyz;
    state.setQuaternion(tmp);
}

lama::Pose3D::Pose3D(const Matrix4d& transformation)
    : state(transformation)
{}

lama::Pose3D::Pose3D(const Affine3d& transformation)
{
    state.translation() = transformation.translation();
    state.setRotationMatrix(transformation.linear());
}

lama::Pose3D::Pose3D(const Pose3D& other)
{
    state = other.state;
}

lama::Pose3D::Pose3D(const SE3d& se3)
    : state(se3)
{}

lama::Pose3D::~Pose3D()
{}

lama::Pose3D lama::Pose3D::operator+(const Pose3D& other)
{
    return Pose3D(state * other.state);
}

lama::Pose3D lama::Pose3D::operator-(const Pose3D& other)
{
    return Pose3D(state.inverse() * other.state);
}

lama::Pose3D& lama::Pose3D::operator+=(const Pose3D& other)
{
    state *= other.state;
    return *this;
}

lama::Pose3D& lama::Pose3D::operator-=(const Pose3D& other)
{
    state = state.inverse() * other.state;
    return *this;
}

Eigen::Vector3d lama::Pose3D::operator*(const Vector3d& point)
{
    return state*point;
}

double lama::Pose3D::x() const
{
    return state.translation().x();
}

double lama::Pose3D::y() const
{
    return state.translation().y();
}

double lama::Pose3D::z() const
{
    return state.translation().z();
}

Eigen::Vector3d lama::Pose3D::xyz() const
{
    return state.translation();
}

double lama::Pose3D::roll() const
{
    return rpy().x();
}

double lama::Pose3D::pitch() const
{
    return rpy().y();
}

double lama::Pose3D::yaw() const
{
    return rpy().z();
}

Eigen::Vector3d lama::Pose3D::rpy() const
{
    return state.rotationMatrix().eulerAngles(0,1,2);
}

