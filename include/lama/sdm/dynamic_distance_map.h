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

#include <queue>

#include "distance_map.h"

namespace lama {

/**
 * Dynamic version of a distance map.
 */
class DynamicDistanceMap : public DistanceMap {
public:

    struct distance_t {
        Vector3s obstacle;
        uint16_t sqdist;
        bool valid_obstacle;
        bool is_queued;
    };

    DynamicDistanceMap(const DynamicDistanceMap& other);

    DynamicDistanceMap(double resolution, uint32_t patch_size = 32, bool is3d = false);
    virtual ~DynamicDistanceMap();

    double distance(const Vector3d& coordinates, Vector3d* gradient = 0) const;
    double distance(const Vector3ui& coordinates) const;

    void addObstacle(const Vector3ui& location);
    void removeObstacle(const Vector3ui& location);

    uint32_t update();

    void setMaxDistance(double distance);
    double maxDistance() const;

protected:

    /**
     * Write internal parameters of the map.
     */
    void writeParameters(std::ofstream& stream) const;

    /**
     * Read internal parameters of the map.
     */
    void readParameters(std::ifstream& stream);

private:

    void raise(const Vector3ui& location, distance_t& current);
    void lower(const Vector3ui& location, distance_t& current);

private:

    typedef std::pair<int, Vector3ui> queue_pair_t;

    struct compare_prio {
        inline bool operator()(const queue_pair_t& left, const queue_pair_t& right)
        { return left.first > right.first; }
    };

    typedef std::priority_queue<queue_pair_t,
            std::vector<queue_pair_t>, compare_prio> queue_t;

    queue_t lower_;
    queue_t raise_;

    int deltas_[26][3];
    uint32_t max_sqdist_;

};

} /* lama */

