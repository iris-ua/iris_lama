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

#include "lama/sdm/simple_occupancy_map.h"

lama::SimpleOccupancyMap::SimpleOccupancyMap(double resolution, uint32_t patch_size, bool is3d)
    : OccupancyMap(resolution, sizeof(char), patch_size, is3d)
{}

lama::SimpleOccupancyMap::SimpleOccupancyMap(const SimpleOccupancyMap& other)
    : OccupancyMap(other)
{}

lama::SimpleOccupancyMap::~SimpleOccupancyMap()
{}

bool lama::SimpleOccupancyMap::setFree(const Vector3d& coordinates)
{
    return setFree(w2m(coordinates));
}

bool lama::SimpleOccupancyMap::setFree(const Vector3ui& coordinates)
{
    char* cell = (char*) get(coordinates);
    if (*cell == -1)
        return false;

    *cell = -1;
    return true;
}

bool lama::SimpleOccupancyMap::setOccupied(const Vector3d& coordinates)
{
    return setOccupied(w2m(coordinates));
}

bool lama::SimpleOccupancyMap::setOccupied(const Vector3ui& coordinates)
{
    char* cell = (char*) get(coordinates);
    if (*cell == 1)
        return false;

    *cell = 1;
    return true;
}

bool lama::SimpleOccupancyMap::setUnknown(const Vector3d& coordinates)
{
    return setUnknown(w2m(coordinates));
}

bool lama::SimpleOccupancyMap::setUnknown(const Vector3ui& coordinates)
{
    char* cell = (char*) get(coordinates);
    if ( *cell == 0 )
        return false;

    *cell = 0;
    return true;
}

bool lama::SimpleOccupancyMap::isFree(const Vector3d& coordinates) const
{
    return isFree(w2m(coordinates));
}

bool lama::SimpleOccupancyMap::isFree(const Vector3ui& coordinates) const
{
    const char* cell = (const char*) get(coordinates);
    if (cell == 0)
        return false;

    return *cell == -1;
}

bool lama::SimpleOccupancyMap::isOccupied(const Vector3d& coordinates) const
{
    return isOccupied(w2m(coordinates));
}

bool lama::SimpleOccupancyMap::isOccupied(const Vector3ui& coordinates) const
{
    const char* cell = (const char*) get(coordinates);
    if (cell == 0)
        return false;

    return *cell == 1;
}

bool lama::SimpleOccupancyMap::isUnknown(const Vector3d& coordinates) const
{
    return isUnknown(w2m(coordinates));
}

bool lama::SimpleOccupancyMap::isUnknown(const Vector3ui& coordinates) const
{
    const char* cell = (const char*) get(coordinates);
    if (cell == 0) return true;

    return *cell == 0;
}

double lama::SimpleOccupancyMap::getProbability(const Vector3d& coordinates) const
{
    return getProbability(w2m(coordinates));
}

double lama::SimpleOccupancyMap::getProbability(const Vector3ui& coordinates) const
{
    const char* cell = (const char*) get(coordinates);
    if (cell == 0) return 0.5;

    switch(*cell){
        case -1: return 0.0;
        case  1: return 1.0;
        default: return 0.5;
    }
}

