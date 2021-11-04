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

#include <stdint.h>

#include <memory>
#include <vector>
#include <array>
#include <deque>
#include <list>
#include <map>
#include <set>

#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Eigen {

typedef Matrix<int16_t,  3, 1> Vector3s;
typedef Matrix<uint32_t, 3, 1> Vector3ui;
typedef Matrix<int64_t,  3, 1> Vector3l;
typedef Matrix<uint64_t, 3, 1> Vector3ul;

typedef std::deque<Vector2d, aligned_allocator<Vector2d>>  VectorVector2d;
typedef std::deque<Vector3d, aligned_allocator<Vector3d>>  VectorVector3d;
typedef std::deque<Vector3ui,aligned_allocator<Vector3ui>> VectorVector3ui;
typedef std::deque<VectorXd, aligned_allocator<VectorXd>>  VectorVectorXd;

}

namespace lama {

template<class T>
using List = std::deque<T>;

template<class T>
using LinkedList = std::list<T>;

// A vector is an algebraic construction, not a container.
// What this container actually represents is an array
// with a continuous memory segment with dynamic size.
template<class T>
using DynamicArray = std::vector<T>;

template<class T, std::size_t N>
using Array = std::array<T, N>;

template<class Key, class T>
using Dictionary = std::map<Key, T>;

//--

using namespace Eigen;

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

struct KeyHash {
    size_t operator()(const Vector3ui& key) const
    { return key(2) + 2642244ul * ( key(1) + key(0) * 2642244ul); }
};
typedef std::unordered_set<Vector3ui, KeyHash> KeySet;

struct PointCloudXYZ {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<PointCloudXYZ> Ptr;

    std::vector<Vector3d> points;

    Vector3d    sensor_origin_;
    Quaterniond sensor_orientation_;
};

struct PolygonMesh {
    List<Vector3f> vertex;
    List<Vector3f> normal;
    List<Vector3f> color;
    List<uint32_t> index;
};

} /* lama */

