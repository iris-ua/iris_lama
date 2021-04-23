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

#include "lama/sdm/dynamic_distance_map.h"

lama::DynamicDistanceMap::DynamicDistanceMap(double resolution, uint32_t patch_size, bool is3d)
    : DistanceMap(resolution, sizeof(distance_t), patch_size, is3d),
      max_sqdist_(100)
{
    int idx = 0;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y){

            if (x == 0 && y == 0)
                continue;

            deltas_[idx][0] = x;
            deltas_[idx][1] = y;
            deltas_[idx][2] = 0;

            idx++;
        }

    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            for (int z = -1; z <= 1; z += 2){

                deltas_[idx][0] = x;
                deltas_[idx][1] = y;
                deltas_[idx][2] = z;

                idx++;
            }

}

lama::DynamicDistanceMap::DynamicDistanceMap(const DynamicDistanceMap& other)
    : DistanceMap(other)
{

    max_sqdist_ = other.max_sqdist_;

    int idx = 0;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y){

            if (x == 0 && y == 0)
                continue;

            deltas_[idx][0] = x;
            deltas_[idx][1] = y;
            deltas_[idx][2] = 0;

            idx++;
        }

    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            for (int z = -1; z <= 1; z += 2){

                deltas_[idx][0] = x;
                deltas_[idx][1] = y;
                deltas_[idx][2] = z;

                idx++;
            }
}

lama::DynamicDistanceMap::~DynamicDistanceMap()
{}


double lama::DynamicDistanceMap::distance(const Vector3d& coordinates, Vector3d* gradient) const
{
    Vector3d map_coords = w2m_nocast(coordinates);
    Vector3ui disc_coords(map_coords.cast<unsigned int>());

    Vector3d mu(map_coords - disc_coords.cast<double>());
    Vector3d muinv = Vector3d::Ones() - mu;

    double dist = 0;

    if ( not is_3d ){
        Matrix<double, 4, 1> val;
        val <<  distance(disc_coords),
            distance(Vector3ui(disc_coords + Vector3ui(1, 0, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(0, 1, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(1, 1, 0)));

        dist = val(0) * muinv(0) * muinv(1) +
            val(1) * muinv(1) * mu(0) +
            val(2) * muinv(0) * mu(1) +
            val(3) * mu(0) * mu(1);

        if (gradient)
            *gradient << -((val(0) - val(1)) * muinv(1) + (val(2) - val(3))*mu(1)) * scale,
                         -((val(0) - val(2)) * muinv(0) + (val(1) - val(3))*mu(0)) * scale,
                          0;

    } else {
        Matrix<double, 8, 1> val;
        val <<  distance(disc_coords),
            distance(Vector3ui(disc_coords + Vector3ui(1, 0, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(0, 1, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(1, 1, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(0, 0, 1))),
            distance(Vector3ui(disc_coords + Vector3ui(1, 0, 1))),
            distance(Vector3ui(disc_coords + Vector3ui(0, 1, 1))),
            distance(Vector3ui(disc_coords + Vector3ui(1, 1, 1)));

        dist = val(0) * muinv.prod() +                  // V000
            val(1) * mu(0) * muinv(1) * muinv(2) +      // V100
            val(2) * muinv(0) * mu(1) * muinv(2) +      // V010
            val(3) * mu(0) * mu(1) * muinv(2) +         // V110
            val(4) * muinv(0) * muinv(1) * mu(2) +      // V001
            val(5) * mu(0) * muinv(1) * mu(2) +         // V101
            val(6) * muinv(0) * mu(1) * mu(2) +         // V011
            val(7) * mu.prod();                         // V111

        if (gradient){

            double a, b;
            // X gradient
            a = (val(0) - val(1)) * muinv(1) + (val(2) - val(3)) * mu(1);
            b = (val(4) - val(5)) * muinv(1) + (val(6) - val(7)) * mu(1);
            (*gradient)[0] = -(a * muinv(2) + b * mu(2)) * scale;

            // ------------------
            // Y gradient
            a = (val(0) - val(2)) * muinv(0) + (val(1) - val(3)) * mu(0);
            b = (val(4) - val(6)) * muinv(0) + (val(5) - val(7)) * mu(0);
            (*gradient)[1] = -(a * muinv(2) + b * mu(2)) * scale;

            // ------------------
            // Z gradient
            a = (val(0) - val(4)) * muinv(0) + (val(1) - val(5)) * mu(0);
            b = (val(2) - val(6)) * muinv(0) + (val(3) - val(7)) * mu(0);
            (*gradient)[2] = -(a * muinv(1) + b * mu(1)) * scale;

        }
    }


    return dist;
}

double lama::DynamicDistanceMap::distance(const Vector3ui& coordinates) const
{
    const distance_t* cell = (const distance_t*) get(coordinates);
    if (cell == 0 || not cell->valid_obstacle)
        return std::sqrt((double)max_sqdist_) * resolution;

    return std::sqrt((double)cell->sqdist) * resolution;
}

void lama::DynamicDistanceMap::setMaxDistance(double distance)
{
    max_sqdist_  = std::ceil( distance * scale );
    max_sqdist_ *= max_sqdist_;
}

double lama::DynamicDistanceMap::maxDistance() const
{
    return std::sqrt((double)(max_sqdist_)) * resolution;
}

uint32_t lama::DynamicDistanceMap::update()
{
    uint32_t number_of_proccessed_cells = 0;

    while (not raise_.empty()){
        Vector3ui location  = raise_.top().second; raise_.pop();
        distance_t* current = (distance_t*) get(location);

        if(current == 0)
            continue;

        ++number_of_proccessed_cells;

        raise(location, *current);
    }// end while

    while (not lower_.empty()){
        Vector3ui location  = lower_.top().second; lower_.pop();
        distance_t* current = (distance_t*)get(location);

        ++number_of_proccessed_cells;

        if(current == 0)
            continue;

        if (current->valid_obstacle) {
            Vector3ui obs = (location.cast<int64_t>() + current->obstacle.cast<int64_t>()).cast<uint32_t>();
            const distance_t* obstacle = (distance_t*) get(obs);
            if (obstacle == 0)
                continue;

            if (obstacle->sqdist == 0)
                lower(location, *current);
        } // end if
    }// end while

    return number_of_proccessed_cells;
}

//=====
void lama::DynamicDistanceMap::writeParameters(std::ofstream& stream) const
{
    stream.write((char*)&max_sqdist_, sizeof(max_sqdist_));
}

void lama::DynamicDistanceMap::readParameters(std::ifstream& stream)
{
    stream.read((char*)&max_sqdist_, sizeof(max_sqdist_));
}

//==================================================================================================
//
void lama::DynamicDistanceMap::addObstacle(const Vector3ui& location)
{
    distance_t* cell = (distance_t*) get(location);
    if (cell == nullptr) return;

    if ((cell->valid_obstacle and cell->sqdist == 0) )
        return; // already an obstacle or already in process of update

    cell->sqdist         = 0;
    cell->obstacle       = Vector3s::Zero();
    cell->valid_obstacle = true;
    cell->is_queued      = true;

    lower_.push({0, location});
}

void lama::DynamicDistanceMap::removeObstacle(const Vector3ui& location)
{
    distance_t* cell = (distance_t*) get(location);
    if (cell == nullptr) return;

    if (not (cell->valid_obstacle and cell->sqdist == 0) )
        return; // already not an obstacle or already in process of update

    cell->sqdist         = 0;
    cell->obstacle       = Vector3s::Zero();
    cell->valid_obstacle = false;
    cell->is_queued      = true;

    raise_.push({0, location});
}

void lama::DynamicDistanceMap::raise(const Vector3ui& location, distance_t& current)
{
    Vector3ui newloc;
    const int numOfNeighbor = is_3d ? 26 : 8;
    for (int i = 0; i < numOfNeighbor; ++i){

        Vector3l d(deltas_[i][0], deltas_[i][1], deltas_[i][2]);

        newloc = (location.cast<int64_t>() + d).cast<uint32_t>();
        distance_t* neighbor = (distance_t*) get(newloc);
        if (neighbor == 0 or neighbor->is_queued)
            continue;

        if (not neighbor->valid_obstacle)
            continue;

        Vector3ui obs = (newloc.cast<int64_t>() + neighbor->obstacle.cast<int64_t>()).cast<uint32_t>();
        const distance_t* obstacle = (distance_t*) get(obs);
        if (obstacle == 0)
            continue;

        // verify if the closest point is no longer an obstacle.
        if (not (obstacle->valid_obstacle)){
            raise_.push({neighbor->sqdist, newloc});

            neighbor->sqdist         = 0;
            neighbor->obstacle       = Vector3s::Zero();
            neighbor->valid_obstacle = false;
            neighbor->is_queued      = true;
        }else if(not neighbor->is_queued){
            lower_.push({neighbor->sqdist, newloc});

            neighbor->is_queued = true;
        }

    }// end for

    current.is_queued = false;
}

void lama::DynamicDistanceMap::lower(const Vector3ui& location, distance_t& current)
{
    if (not current.is_queued)
        // This happens when a cell already in the queue is affected by a closer obstacle.
        // Therefore, we can safely skip this one because it was already processed.
        return;

    Vector3l newloc;
    const int numOfNeighbor = is_3d ? 26 : 8;
    for (int i = 0; i < numOfNeighbor; ++i){

        // deltas
        Vector3l d(deltas_[i][0], deltas_[i][1], deltas_[i][2]);

        // only update away from the obstacle
        if ( ( d.cwiseProduct(current.obstacle.cast<int64_t>()).array() > 0).any() )
            continue;

        newloc = (location.cast<int64_t>() + d);
        distance_t* neighbor = (distance_t*) get(newloc.cast<uint32_t>());
        if (neighbor == 0)
            continue;

        // Absolute position of the current obstacle
        Vector3l obs = (location.cast<int64_t>() + current.obstacle.cast<int64_t>());

        Vector3l dist = (newloc - obs);
        uint32_t new_sqdist = dist.squaredNorm();
        uint32_t cmp_sqdist = neighbor->valid_obstacle ? neighbor->sqdist : max_sqdist_;

        bool overwrite = (new_sqdist < cmp_sqdist);

        if(not overwrite && new_sqdist == neighbor->sqdist){
            // Absolute position of the neighbor obstacle
            Vector3ui nobs = (newloc + neighbor->obstacle.cast<int64_t>()).cast<uint32_t>();
            const distance_t* obstacle = (distance_t*) get(nobs);
            if (not neighbor->valid_obstacle || (obstacle != 0 && not (obstacle->valid_obstacle and obstacle->sqdist == 0) ))
                overwrite = true;
        }

        if ( overwrite ){
            lower_.push({new_sqdist, newloc.cast<uint32_t>()});

            neighbor->sqdist         = new_sqdist;
            neighbor->valid_obstacle = true;
            neighbor->obstacle       = (obs - newloc).cast<int16_t>();
            neighbor->is_queued      = true;
        }
    }// end for

    current.is_queued = false;
}

