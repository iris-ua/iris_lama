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

#include <lama/pose2d.h>

lama::Pose2D::Pose2D()
    : state()
{}

lama::Pose2D::Pose2D(const double& x, const double& y, const double& rotation)
    : state(rotation, Vector2d(x,y))
{}

lama::Pose2D::Pose2D(const Vector2d& xy, const double& rotation)
    : state(rotation, xy)
{}

lama::Pose2D::Pose2D(const Vector3d& xyr)
    : state(xyr[2], xyr.tail<2>())
{}

lama::Pose2D::Pose2D(const Affine2d& transformation)
{
    state.translation() = transformation.translation();
    state.setRotationMatrix(transformation.linear());
}

lama::Pose2D::Pose2D(const Pose2D& other)
{
    state = other.state;
}

lama::Pose2D::Pose2D(const SE2d& se2)
    : state(se2)
{}

lama::Pose2D::~Pose2D()
{}

lama::Pose2D& lama::Pose2D::operator=(const lama::Pose2D& other)
{
    state = other.state;
    return *this;
}

lama::Pose2D lama::Pose2D::operator+(const lama::Pose2D& other)
{
    return Pose2D(state * other.state);
}

lama::Pose2D lama::Pose2D::operator-(const lama::Pose2D& other)
{
    return Pose2D(state.inverse() * other.state);
}

lama::Pose2D& lama::Pose2D::operator+=(const lama::Pose2D& other)
{
    state *= other.state;
    return *this;
}

lama::Pose2D& lama::Pose2D::operator-=(const lama::Pose2D& other)
{
    state = state.inverse() * other.state;
    return *this;
}

lama::Vector2d lama::Pose2D::operator*(const Vector2d& point)
{
    return state*point;
}

double lama::Pose2D::x() const
{
    return state.translation().x();
}

double lama::Pose2D::y() const
{
    return state.translation().y();
}

Eigen::Vector2d lama::Pose2D::xy() const
{
    return state.translation();
}

double lama::Pose2D::rotation() const
{
    return state.so2().log();
}

const Eigen::Vector3d lama::Pose2D::xyr() const
{
    Vector3d tmp;
    tmp.x() = state.translation().x();
    tmp.y() = state.translation().y();
    tmp.z() = state.so2().log();
    return tmp;
}

