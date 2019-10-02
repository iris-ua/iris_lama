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

#include <limits>
#include <fstream>

#include "lama/print.h"
#include "lama/buffer_compressor.h"

#include "lama/sdm/map.h"

lama::Map::Map(double resolution, size_t cell_size, uint32_t patch_size, bool is3d) :
    resolution(resolution), scale(1.0/ resolution),
    cell_memory_size(cell_size),
    patch_length(patch_size),
    patch_volume( patch_size * patch_size * (is3d ? patch_size : 1.0)),
    is_3d(is3d)
{
    // Having always positive map coordinates simplifies the access to the
    // data. Therefore we adjust the origin of the world to be the half the maximum
    // positive map coordinate.
    Vector3d adjust;
    adjust.fill(UNIVERSAL_CONSTANT / 2);

    tf_     = Translation3d(adjust * patch_length) * Scaling(scale);
    tf_inv_ = tf_.inverse();

    //
    buffer_ = 0;
    lru_max_size_ = 160;
    use_compression_ = false;
    cache_miss_ = 0;
    cache_hit_ = 0;

    bc_ = 0; // compression

}

lama::Map::Map(const Map& other) :
    resolution(other.resolution), scale(other.scale),
    cell_memory_size(other.cell_memory_size),
    patch_length(other.patch_length),
    patch_volume(other.patch_volume),
    is_3d(other.is_3d)
{
    tf_ = other.tf_;
    tf_inv_ = other.tf_inv_;

    use_compression_ = other.use_compression_;

    if (use_compression_){
        lru_max_size_ = other.lru_max_size_;
        bc_ = other.bc_->clone();
        buffer_ = new char[bc_->compressBound(patch_volume * cell_memory_size)];
    } else{
        buffer_ = 0;
        bc_ = 0;
    }

    for (auto& kv : other.patches)
        patches[kv.first] = COWPtr< Container >(kv.second);

    // rebuild the lru
    list_iterator_t lru_it = other.lru_items_list_.begin();
    list_iterator_t lru_it_end = other.lru_items_list_.end();

    for (; lru_it != lru_it_end; ++lru_it){
        lru_items_list_.push_back(key_value_pair_t(lru_it->first, &(patches[lru_it->first])));
        lru_items_map_[lru_it->first] = --lru_items_list_.end();
    }
}

lama::Map::~Map()
{
    delete bc_;
    delete [] buffer_;
}

size_t lama::Map::memory() const
{
    double total = 0.0;

    for(auto& kv : patches){
        total += sizeof(uint64_t) + sizeof(COWPtr< Container >) + sizeof(Container*);
        total += (double)kv.second->memory() / (double)kv.second.use_count();
    }

    return (size_t)total;
}

size_t lama::Map::fullMemory() const
{
    size_t total = 0;

    for(auto& kv : patches){
        total += sizeof(uint64_t) + sizeof(COWPtr< Container >) + sizeof(Container*);
        total += kv.second->fullMemory() / kv.second.use_count();
    }

    return total;
}

void lama::Map::bounds(Vector3ui& min, Vector3ui& max) const
{
    min.fill(std::numeric_limits<Vector3ui::Scalar>::max());
    max.fill(std::numeric_limits<Vector3ui::Scalar>::min());

    for (auto& it : patches){
        auto id = it.first;
        Vector3ui anchor = unhash(id, UNIVERSAL_CONSTANT) * patch_length;

        min(0) = std::min(min(0), anchor(0));
        min(1) = std::min(min(1), anchor(1));
        min(2) = std::min(min(2), anchor(2));

        max(0) = std::max(max(0), anchor(0));
        max(1) = std::max(max(1), anchor(1));
        max(2) = std::max(max(2), anchor(2));
    }// end for_all

    max.array() += patch_length;
}

bool lama::Map::patchAllocated(const Vector3ui& coordinates) const
{
    return (get(coordinates) != 0);
}

bool lama::Map::patchIsUnique(const Vector3ui& coordinates) const
{
    uint64_t idx; // patch index
    Vector3ul rcd = (coordinates / patch_length).template cast<uint64_t>();

    if (is_3d)
        idx = rcd(2) + 2642244ul * ( rcd(1) + rcd(0) * 2642244ul);
    else
        idx = rcd(1) + rcd(0) * 2642244ul;

    auto it = patches.find(idx);
    if (it == patches.end())
        return true;

    return it->second.unique();
}

void lama::Map::useCompression(bool compression, uint32_t lru_size, const std::string& algorithm)
{
    use_compression_ = compression;
    lru_max_size_ = lru_size;
    if (use_compression_){

        delete bc_;

        if (algorithm == "zstd")
            bc_ = new ZSTDBufferCompressor;
        else
            bc_ = new LZ4BufferCompressor;

        buffer_ = new char[bc_->compressBound(patch_volume * cell_memory_size )];

    } else {
        delete [] buffer_;
        buffer_ = 0;
        delete bc_;
    }

}

void lama::Map::computeRay(const Vector3ui& from, const Vector3ui& to, VectorVector3ui& sink)
{
    if ( from == to ) return;

    Vector3l error = Vector3l::Zero();
    Vector3l coord = from.cast<int64_t>();
    Vector3l delta = to.cast<int64_t>() - coord;

    Vector3l step = (delta.array() < 0).select(-1, Vector3l::Ones());

    delta = delta.array().abs();
    int n = delta.maxCoeff() - 1;

    // maximum change of any coordinate
    for (int i = 0; i < n; ++i){
        // update errors
        error += delta;

        for (int j = 0; j < 3; ++j)
            if ( (error(j) << 1) >= n ){
                coord(j) += step(j);
                error(j) -= n;
            }

        // save the coordinate
        sink.push_back(coord.cast<uint32_t>() );
    }
}

void lama::Map::computeRay(const Vector3d& from, const Vector3d& to, VectorVector3ui& sink)
{
    // NOTICE: This function is borrowed from
    // https://github.com/OctoMap/octomap/blob/devel/octomap/include/octomap/OcTreeBaseImpl.hxx
    // All credits goes to the authors of Octomap.

    if ( from == to ) return;

    // == Initialization phase ==
    Vector3d direction = (to - from);
    double length = direction.norm();
    direction.normalize();

    int    step[3];
    double tMax[3];
    double tDelta[3];

    Vector3ui current_key = this->w2m(from);
    Vector3ui end_key     = this->w2m(to);
    Vector3d  vb = this->m2w(current_key);

    for(unsigned int i=0; i < 3; ++i) {
        // compute step direction
        if (direction(i) > 0.0) step[i] =  1;
        else if (direction(i) < 0.0)   step[i] = -1;
        else step[i] = 0;

        // compute tMax, tDelta
        if (step[i] != 0) {
            // corner point of voxel (in direction of ray)
            double voxelBorder = vb(i);
            voxelBorder += (float) (step[i] * this->resolution * 0.5);

            tMax[i] = ( voxelBorder - from(i) ) / direction(i);
            tDelta[i] = this->resolution / std::fabs( direction(i) );
        }
        else {
            tMax[i] =  std::numeric_limits<double>::max( );
            tDelta[i] = std::numeric_limits<double>::max( );
        }
    }

    // == Incremental phase ==
    bool done = false;
    while (!done) {

        unsigned int dim;

        // find minimum tMax:
        if (tMax[0] < tMax[1]){
            if (tMax[0] < tMax[2]) dim = 0;
            else                   dim = 2;
        }
        else {
            if (tMax[1] < tMax[2]) dim = 1;
            else                   dim = 2;
        }

        // advance in direction "dim"
        current_key[dim] += step[dim];
        tMax[dim] += tDelta[dim];

        // reached endpoint, key equv?
        if (current_key == end_key) {
            done = true;
            break;
        }
        else {

            // reached endpoint world coords?
            // dist_from_origin now contains the length of the ray when
            // traveled until the border of the current voxel.
            double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);

            // if this is longer than the expected ray length, we should have
            // already hit the voxel containing the end point with the code
            // above (key_end). However, we did not hit it due to accumulating
            // discretization errors, so this is the point here to stop the
            // ray as we would never reach the voxel key_end
            if (dist_from_origin > length) {
                done = true;
                break;
            }

            else {  // continue to add freespace cells
                sink.push_back(current_key);
            }
        }
    } // end while

}

void lama::Map::visit_all_cells(const CellWalker& walker)
{
    for (auto& it : patches){
        auto id = it.first;
        Vector3ui anchor = unhash(id, UNIVERSAL_CONSTANT) * patch_length;

        for (uint32_t idx = 0; idx < patch_volume; ++idx)
            walker(anchor + unhash(idx, patch_length));
    }// end for_all
}

void lama::Map::visit_all_cells(const CellWalker& walker) const
{
    for (auto& it : patches){
        auto id = it.first;
        Vector3ui anchor = unhash(id, UNIVERSAL_CONSTANT) * patch_length;

        for (uint32_t idx = 0; idx < patch_volume; ++idx)
            walker(anchor + unhash(idx, patch_length));
    }// end for_all
}

// == Protected ====================================================================================

uint8_t* lama::Map::get(const Vector3ui& coordinates)
{
    uint64_t idx; // patch index
    Vector3ul rcd = (coordinates / patch_length).template cast<uint64_t>();

    if (is_3d)
        idx = rcd(2) + 2642244ul * ( rcd(1) + rcd(0) * 2642244ul);
    else
        idx = rcd(1) + rcd(0) * 2642244ul;

    // get the container
    COWPtr< Container >* p;

    if (use_compression_){
        p = lru_get(idx);

        if (p == 0){
            // Not in the cache
            auto it = patches.find(idx);
            if (it == patches.end()){
                // first time reference
                it = patches.insert(std::make_pair(idx, COWPtr< Container >(new Container)) ).first;
                //it->second->alloc(patch_volume_);
                it->second->alloc(patch_volume, cell_memory_size);

                p = &(it->second);
                cache_miss_--; // a brand new patch is not a cache miss
            } else {
                p = &(it->second);
                (*p)->decompress(bc_);
            }

            lru_put(idx, p);
        }

    } else {

        auto it = patches.find(idx);
        if (it == patches.end()){
            it = patches.insert(std::make_pair(idx, COWPtr< Container >(new Container)) ).first;
            //it->second->alloc(patch_volume_);
            it->second->alloc(patch_volume, cell_memory_size);
        }

        p = &(it->second);
    }

    rcd  = coordinates.cast<uint64_t>() - rcd * (uint64_t) patch_length;

    if (is_3d)
        idx = rcd(2) + patch_length * (rcd(1) + rcd(0) * patch_length);
    else
        idx = rcd(1) + rcd(0) * patch_length;

    return (*p)->get(idx);
}

const uint8_t* lama::Map::get(const Vector3ui& coordinates) const
{
    uint64_t idx; // patch index
    Vector3ul rcd = (coordinates / patch_length).cast<uint64_t>();

    if (is_3d)
        idx = rcd(2) + 2642244ul * ( rcd(1) + rcd(0) * 2642244ul);
    else
        idx = rcd(1) + rcd(0) * 2642244ul;

    COWPtr< Container >* p;

    if (use_compression_){
        p = lru_get(idx);

        // is it in cache ?
        if (p == 0){
            auto it = patches.find(idx);
            if (it == patches.end()){
                return 0;
            } else {

                p = const_cast<COWPtr< Container >* >(&(it->second));
                (*p)->decompress(bc_);
            }

            lru_put(idx, p);
        }

    } else {

        auto it = patches.find(idx);
        if (it == patches.end())
            return 0;

        p = const_cast<COWPtr< Container >* >(&(it->second));
    }

    rcd = coordinates.cast<uint64_t>() - rcd * patch_length;

    if (is_3d)
        idx = rcd(2) + patch_length * (rcd(1) + rcd(0) * patch_length);
    else
        idx = rcd(1) + rcd(0) * patch_length;


    // immutable version is called because p has
    // a const qualifier
    return (*p).read_only()->get(idx);
}

uint64_t lama::Map::hash(const Vector3ui& coordinates) const
{
    if (is_3d)
        return coordinates(2) + 2642244ul * ( coordinates(1) + coordinates(0) * 2642244ul);

    return coordinates(1) + coordinates(0) * 2642244ul;
}

bool lama::Map::write(const std::string& filename) const
{
#if 0
    std::ofstream f(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);

    if (not f.is_open())
        return false;

    const uint32_t magic = 0x00285145;
    const uint32_t Tsize = sizeof(T);
    const uint32_t Psize = patches_.size(); // ensure type

    f.write((char*)&magic,       sizeof(magic)); // magic number
    f.write((char*)&Tsize,       sizeof(Tsize));   // the size of each element

    f.write((char*)&resolution_, sizeof(resolution_));
    f.write((char*)&patch_size_, sizeof(patch_size_));
    f.write((char*)&is3d_,       sizeof(is3d_));

    f.write((char*)&Psize, sizeof(Psize)); // total number of patches

    typename PatchMap::const_iterator it = patches_.begin();
    for (; it != patches_.end(); ++it){

        f.write((char*)&(it->first), sizeof(it->first));
        it->second->write(bc_, f);

    }// end for

    // write map defined parameters
    this->writeParameters(f);

    f.close();
    return true;
#endif
    return false;
}

bool lama::Map::read(const std::string& filename)
{
#if 0
    std::ifstream f(filename.c_str(), std::ios::in | std::ios::binary);

    if (not f.is_open())
        return false;

    uint32_t magic;
    uint32_t Tsize;

    f.read((char*)&magic, sizeof(magic)); // magic number
    f.read((char*)&Tsize, sizeof(Tsize)); // the size of each element

    // For a valid map, magic number and the sizeof of an element
    // must be equal to expected values.
    if (magic != 0x00285145 or Tsize != sizeof(T))
        return false;

    f.read((char*)&resolution_, sizeof(resolution_));
    f.read((char*)&patch_size_, sizeof(patch_size_));
    f.read((char*)&is3d_,       sizeof(is3d_));

    if (is3d_)
        patch_volume_ = patch_size_ * patch_size_ * patch_size_;
    else
        patch_volume_ = patch_size_ * patch_size_;

    Vector3d adjust;
    adjust.fill(2642244 / 2);

    scale_ = 1.0 / resolution_;
    tf_     = Translation3d(adjust * patch_size_) * Scaling(scale_);
    tf_inv_ = tf_.inverse();

    uint32_t numOfPatches;
    f.read((char*)&numOfPatches, sizeof(numOfPatches)); // total number of patches

    typename PatchMap::iterator it;
    uint64_t idx;
    for (size_t i = 0; i < numOfPatches; ++i){

        f.read((char*)&idx, sizeof(idx));

        it = patches_.insert(std::make_pair(idx, COWPtr< Container<T> >(new Container<T>)) ).first;
        it->second->alloc(patch_volume_);
        it->second->read(f);

        ldc::ZSTDBufferCompressor zstd;
        it->second->decompress(&zstd);

        if (use_compression_)
            it->second->compress(bc_, buffer_);

    }// end for

    // read map defined parameters
    this->readParameters(f);

    return true;
#endif
    return false;
}

//==============================================================================
Eigen::Vector3ui lama::Map::unhash(uint64_t idx, uint64_t stride) const
{
    if (is_3d)
        return Vector3ui(idx / (stride*stride),
                         idx % (stride*stride) / stride,
                         idx % (stride*stride) % stride);
    // else
    return Vector3ui(idx / stride, idx % stride, 0);
}

bool lama::Map::lru_key_exists(uint64_t idx) const
{
    return lru_items_map_.find(idx) != lru_items_map_.end();
}

void lama::Map::lru_put(uint64_t idx, COWPtr< Container >* container) const
{
    lru_items_list_.push_front(key_value_pair_t(idx, container));

    cache_miss_++;

    lru_items_map_[idx] = lru_items_list_.begin();

    if (lru_items_map_.size() > lru_max_size_){
        list_iterator_t lit = lru_items_list_.end();
        --lit;

        (*(lit->second))->compress(bc_, buffer_);

        lru_items_map_.erase(lit->first);
        lru_items_list_.pop_back();
    }
}

lama::COWPtr< lama::Container >* lama::Map::lru_get(uint64_t idx) const
{
    std::map<uint64_t, list_iterator_t>::const_iterator it = lru_items_map_.find(idx);
    if (it == lru_items_map_.end()){
        return 0;
    }

    cache_hit_++;

    lru_items_list_.splice(lru_items_list_.begin(), lru_items_list_, it->second);
    return it->second->second;
}

