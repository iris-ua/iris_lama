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

#include "occupancy_map.h"

namespace lama {

class SimpleOccupancyMap : public OccupancyMap {
public:

    SimpleOccupancyMap(double resolution, uint32_t patch_size = 32, bool is3d = false);
    SimpleOccupancyMap(const SimpleOccupancyMap& other);

    virtual ~SimpleOccupancyMap();

    bool setFree(const Vector3d& coordinates);
    bool setFree(const Vector3ui& coordinates);

    bool setOccupied(const Vector3d& coordinates);
    bool setOccupied(const Vector3ui& coordinates);

    bool setUnknown(const Vector3d& coordinates);
    bool setUnknown(const Vector3ui& coordinates);

    bool isFree(const Vector3d& coordinates) const;
    bool isFree(const Vector3ui& coordinates) const;

    bool isOccupied(const Vector3d& coordinates) const;
    bool isOccupied(const Vector3ui& coordinates) const;

    bool isUnknown(const Vector3d& coordinates) const;
    bool isUnknown(const Vector3ui& coordinates) const;

    double getProbability(const Vector3d& coordinates) const;
    double getProbability(const Vector3ui& coordinates) const;

protected:

    /**
     * Write internal parameters of the map.
     */
    void writeParameters(std::ofstream& stream) const
    {}

    /**
     * Read internal parameters of the map.
     */
    void readParameters(std::ifstream& stream)
    {}

private:
    SimpleOccupancyMap& operator=(const SimpleOccupancyMap& other);
};

} /* lama */

