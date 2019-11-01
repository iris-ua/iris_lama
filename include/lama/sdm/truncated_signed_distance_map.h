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

#include "distance_map.h"

namespace lama {

/// Truncated Signed Distance Map.
/// This can be used to generate 3D volume representions of the environment.
struct TruncatedSignedDistanceMap : public DistanceMap {

    struct tsd_t {
        float distance;
        float weight;
    };

    TruncatedSignedDistanceMap(const TruncatedSignedDistanceMap& other);
    TruncatedSignedDistanceMap(double resolution, uint32_t patch_size = 32, bool is3d = false);
    virtual ~TruncatedSignedDistanceMap();

    double distance(const Vector3d& coordinates, Vector3d* gradient = 0) const;
    double distance(const Vector3ui& coordinates) const;

    size_t insertPointCloud(const PointCloudXYZ::Ptr& cloud);
    void integrate(const Vector3d& origin, const Vector3d& hit);

    void setMaxDistance(double distance);
    double maxDistance() const;

    void toMesh(PolygonMesh& mesh) const;

private:

    float maximum_distance_;
    float maximum_weight_;
    float truncate_size_;

    float epsilon_;
    float delta_;
};

} /* lama */


