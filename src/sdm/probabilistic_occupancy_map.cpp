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

#include "lama/sdm/probabilistic_occupancy_map.h"

namespace {

inline float prob(const float& logods)
{
    return 1.0 - 1.0 / (1.0 + std::exp(logods));
}

inline float logods(const float& prob)
{
    return std::log(prob / (1.0 - prob));
}

} // namespace

lama::ProbabilisticOccupancyMap::ProbabilisticOccupancyMap(double resolution, uint32_t patch_size, bool is3d)
    : OccupancyMap(resolution, sizeof(prob_tag), patch_size, is3d)
{
    miss_ = logods(0.4);
    hit_  = logods(0.7);

    clamp_min_ = logods(0.12);
    clamp_max_ = logods(0.97);

    occ_thresh_ = 0.0 *logods(0.5);
}

lama::ProbabilisticOccupancyMap::ProbabilisticOccupancyMap(const ProbabilisticOccupancyMap& other)
    : OccupancyMap(other)
{
    miss_ = other.miss_;
    hit_  = other.hit_;

    clamp_min_ = other.clamp_min_;
    clamp_max_ = other.clamp_max_;

    occ_thresh_ = other.occ_thresh_;
}

lama::ProbabilisticOccupancyMap::~ProbabilisticOccupancyMap()
{}

bool lama::ProbabilisticOccupancyMap::setFree(const Vector3d& coordinates)
{
    return setFree(w2m(coordinates));
}

bool lama::ProbabilisticOccupancyMap::setFree(const Vector3ui& coordinates)
{
    prob_tag* cell = (prob_tag*) get(coordinates);

    bool free = cell->prob < occ_thresh_;
    cell->prob = std::max(cell->prob + miss_, clamp_min_);

    if ( free ) return false;
    else return (cell->prob < occ_thresh_);
}

bool lama::ProbabilisticOccupancyMap::setOccupied(const Vector3d& coordinates)
{
    return setOccupied(w2m(coordinates));
}

bool lama::ProbabilisticOccupancyMap::setOccupied(const Vector3ui& coordinates)
{
    prob_tag* cell = (prob_tag*) get(coordinates);

    bool occupied = cell->prob > occ_thresh_;
    cell->prob = std::min(cell->prob + hit_, clamp_max_);

    if ( occupied ) return false;
    else return (cell->prob > occ_thresh_);
}

bool lama::ProbabilisticOccupancyMap::setUnknown(const Vector3d& coordinates)
{
    return setUnknown(w2m(coordinates));
}

bool lama::ProbabilisticOccupancyMap::setUnknown(const Vector3ui& coordinates)
{
    prob_tag* cell = (prob_tag*) get(coordinates);

    bool unknown = cell->prob == occ_thresh_;
    cell->prob = occ_thresh_;

    if ( unknown ) return false;
    else return true;
}

bool lama::ProbabilisticOccupancyMap::isFree(const Vector3d& coordinates) const
{
    return isFree(w2m(coordinates));
}

bool lama::ProbabilisticOccupancyMap::isFree(const Vector3ui& coordinates) const
{
    const prob_tag* cell = (const prob_tag*) get(coordinates);
    if (cell == 0) return false;

    return cell->prob < occ_thresh_;
}

bool lama::ProbabilisticOccupancyMap::isOccupied(const Vector3d& coordinates) const
{
    return isOccupied(w2m(coordinates));
}

bool lama::ProbabilisticOccupancyMap::isOccupied(const Vector3ui& coordinates) const
{
    const prob_tag* cell = (const prob_tag*) get(coordinates);
    if (cell == 0) return false;

    return cell->prob > occ_thresh_;
}

bool lama::ProbabilisticOccupancyMap::isUnknown(const Vector3d& coordinates) const
{
    return isUnknown(w2m(coordinates));
}

bool lama::ProbabilisticOccupancyMap::isUnknown(const Vector3ui& coordinates) const
{
    const prob_tag* cell = (const prob_tag*) get(coordinates);
    if (cell == 0) return true;

    return cell->prob == occ_thresh_;
}

double lama::ProbabilisticOccupancyMap::getProbability(const Vector3d& coordinates) const
{
    return getProbability(w2m(coordinates));
}

double lama::ProbabilisticOccupancyMap::getProbability(const Vector3ui& coordinates) const
{
    const prob_tag* cell = (const prob_tag*) get(coordinates);
    if (cell == 0) return prob(occ_thresh_);

    return prob(cell->prob);
}

