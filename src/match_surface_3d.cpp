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

#include "lama/match_surface_3d.h"

lama::MatchSurface3D::MatchSurface3D(const DynamicDistanceMap*  surface,
                   const PointCloudXYZ::Ptr& scan,
                   const SE3d& estimate)
    : surface_(surface),
      scan_(scan),
      state_(estimate)
{ }

void lama::MatchSurface3D::eval(VectorXd& residuals, MatrixXd* J)
{
    // 1. Transform the point cloud to the model coordinates.
    Affine3d moving_tf = Translation3d(scan_->sensor_origin_) * scan_->sensor_orientation_;
    Affine3d fixed_tf(state_.matrix());

    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);

    const size_t num_points = scan_->points.size();
    //== transform point cloud
    Affine3d tf = fixed_tf * moving_tf;
    cloud->points.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i)
        cloud->points.push_back(tf * scan_->points[i]);
    //==

    // 2. Resize data holders
    Vector3d  hit;
    Vector3d  grad; // will only be used when J != nullptr

    residuals.resize(num_points);
    if (J != 0)
        J->resize(num_points, 6);

    // 3. Compute residuals and Jacobian (if required).
    for (size_t i = 0; i < num_points; ++i){
        hit = cloud->points[i];

        residuals[i] = surface_->distance(hit, &grad);

        if (J != 0){
            // the Jacobian of the euclidean distance (Je) is
            // equal to:
            //
            //     Je = | dx, dy, dz |
            //
            // And the Jacobian of the special euclidean group (Js)
            // that transforms a point is given by :
            //
            //     Js = | 1, 0, 0, 0, z,-y |
            //          | 0, 1, 0,-z, 0, x |
            //          | 0, 0, 1, y,-x, 0 |
            //
            // Then, the final Jacobian is equal to J = Je * Js

            /* Matrix<double, 3, 6> Js; */
            /* Js << 1, 0, 0,       0,  hit[2], -hit[1], */
            /*       0, 1, 0, -hit[2],       0,  hit[0], */
            /*       0, 0, 1,  hit[1], -hit[0],       0; */
            /* J->row(i) = grad.transpose() * Js; */

            J->row(i) << grad[0], grad[1], grad[2],
                         grad[2]*hit[1] - grad[1]*hit[2],
                         grad[0]*hit[2] - grad[2]*hit[0],
                         grad[1]*hit[0] - grad[0]*hit[1];
        }
    }// end for
}

void lama::MatchSurface3D::update(const VectorXd& h)
{
    // The state update in the manifold.
    state_ = SE3d::exp(h) * state_;
}

