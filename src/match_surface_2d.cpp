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

#include "lama/match_surface_2d.h"

lama::MatchSurface2D::MatchSurface2D(const DynamicDistanceMap* surface, const PointCloudXYZ::Ptr& scan, const SE2d& estimate)
    : surface_(surface),
      scan_(scan),
      state_(estimate)
{ }

void lama::MatchSurface2D::eval(VectorXd& residuals, MatrixXd* J)
{
    // 1. Transform the point cloud to the model coordinates.

    // Although we are working on a 2d plane the sensor is on a 3d plane.
    // Thus, to use the data points from the sensor we have project them into the 2d
    // plane of the moving frame and only then to the fixed 2d plane.
    Affine3d moving_tf = Translation3d(scan_->sensor_origin_) * scan_->sensor_orientation_;

    Vector3d trans; trans << state_.translation().x(),
                             state_.translation().y(),
                             0.0;
    Affine3d fixed_tf = Translation3d(trans) * AngleAxisd(state_.so2().log(), Vector3d::UnitZ());

    const size_t num_points = scan_->points.size();
    //== transform point cloud
    Affine3d tf = fixed_tf * moving_tf;

    // 2. Resize data holders
    Vector3d  hit;
    Vector3d  grad; // will only be used when J != nullptr

    residuals.resize(num_points);
    if (J != 0)
        J->resize(num_points, 3);

    // 3. Compute residuals and Jacobian (if required).
    for (size_t i = 0; i < num_points; ++i){
        hit = tf * scan_->points[i];
        hit[2] = 0.0;

        residuals[i] = surface_->distance(hit, &grad);

        if (J != 0)
            // the Jacobian of the euclidean distance (Je) is
            // equal to:
            //
            //     Je = | dx, dy |
            //
            // And the Jacobian of the special euclidean group (Js)
            // that transforms a point is given by :
            //
            //     Js = | 1, 0, -y |
            //          | 0, 1,  x |
            //
            // Then, the final Jacobian is equal to J = Je * Js
            J->row(i) << grad[0], grad[1], grad[1]*hit[0] - grad[0]*hit[1];
    }// end for
}

double lama::MatchSurface2D::error()
{
    // Although we are working on a 2d plane the sensor is on a 3d plane.
    // Thus, to use the data points from the sensor we have project them into the 2d
    // plane of the moving frame and only then to the fixed 2d plane.
    Affine3d moving_tf = Translation3d(scan_->sensor_origin_) * scan_->sensor_orientation_;

    Vector3d trans; trans << state_.translation().x(),
                             state_.translation().y(),
                             0.0;
    Affine3d fixed_tf = Translation3d(trans) * AngleAxisd(state_.so2().log(), Vector3d::UnitZ());

    Affine3d tf = fixed_tf * moving_tf;

    const size_t num_points = scan_->points.size();

    Vector3ui hit;
    VectorXd residuals(num_points);
    for (size_t i = 0; i < num_points; ++i){
        hit = surface_->w2m(tf * scan_->points[i]);
        residuals[i] = surface_->distance(hit);
    }// end for

    return std::sqrt(residuals.squaredNorm() / (residuals.size()));
}

void lama::MatchSurface2D::update(const VectorXd& h)
{
    // The state update in the manifold.
    state_ = SE2d::exp(h) * state_;
}

