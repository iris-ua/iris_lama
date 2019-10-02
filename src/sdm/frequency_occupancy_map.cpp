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

#include "lama/sdm/frequency_occupancy_map.h"

namespace  {

static double occ_thresh = 0.25;

static double prob(const lama::FrequencyOccupancyMap::frequency& freq)
{
    if (freq.visited == 0) return occ_thresh;
    return ((double)freq.occupied) / ((double)freq.visited);
    //return ((double)freq.occupied) / ((double)(freq.visited+freq.occupied));
}

} // namespace

lama::FrequencyOccupancyMap::FrequencyOccupancyMap(double resolution, uint32_t patch_size, bool is3d)
    : OccupancyMap(resolution, sizeof(frequency), patch_size, is3d)
{}

lama::FrequencyOccupancyMap::FrequencyOccupancyMap(const FrequencyOccupancyMap& other)
    : OccupancyMap(other)
{}

lama::FrequencyOccupancyMap::~FrequencyOccupancyMap()
{}

bool lama::FrequencyOccupancyMap::setFree(const Vector3d& coordinates)
{
    return setFree(w2m(coordinates));
}

bool lama::FrequencyOccupancyMap::setFree(const Vector3ui& coordinates)
{
    frequency* cell = (frequency*) get(coordinates);

    bool free = prob(*cell) < occ_thresh;
    cell->visited++;

    if ( free ) return false;
    else return (prob(*cell) < occ_thresh);
}

bool lama::FrequencyOccupancyMap::setOccupied(const Vector3d& coordinates)
{
    return setOccupied(w2m(coordinates));
}

bool lama::FrequencyOccupancyMap::setOccupied(const Vector3ui& coordinates)
{
    frequency* cell = (frequency*) get(coordinates);

    bool occupied = prob(*cell) > occ_thresh;
    cell->occupied++;
    cell->visited++;

    if ( occupied ) return false;
    else return (prob(*cell) > occ_thresh);
}

bool lama::FrequencyOccupancyMap::setUnknown(const Vector3d& coordinates)
{
    return setUnknown(w2m(coordinates));
}

bool lama::FrequencyOccupancyMap::setUnknown(const Vector3ui& coordinates)
{
    frequency* cell = (frequency*) get(coordinates);

    if ( cell->visited == 0 ) return false;

    cell->occupied = 0;
    cell->visited  = 0;

    return true;
}

bool lama::FrequencyOccupancyMap::isFree(const Vector3d& coordinates) const
{
    return isFree(w2m(coordinates));
}

bool lama::FrequencyOccupancyMap::isFree(const Vector3ui& coordinates) const
{
    const frequency* cell = (const frequency*) get(coordinates);
    if ( cell == 0 ) return false;

    return prob(*cell) < occ_thresh;
}

bool lama::FrequencyOccupancyMap::isOccupied(const Vector3d& coordinates) const
{
    return isOccupied(w2m(coordinates));
}

bool lama::FrequencyOccupancyMap::isOccupied(const Vector3ui& coordinates) const
{
    const frequency* cell = (const frequency*) get(coordinates);
    if ( cell == 0 ) return false;

    return prob(*cell) > occ_thresh;
}

bool lama::FrequencyOccupancyMap::isUnknown(const Vector3d& coordinates) const
{
    return isUnknown(w2m(coordinates));
}

bool lama::FrequencyOccupancyMap::isUnknown(const Vector3ui& coordinates) const
{
    const frequency* cell = (const frequency*) get(coordinates);
    if (cell == 0) return true;

    return cell->visited == 0;
}

double lama::FrequencyOccupancyMap::getProbability(const Vector3d& coordinates) const
{
    return getProbability(w2m(coordinates));
}

double lama::FrequencyOccupancyMap::getProbability(const Vector3ui& coordinates) const
{
    const frequency* cell = (const frequency*) get(coordinates);
    if (cell == 0) return occ_thresh;

    return prob(*cell);
}

