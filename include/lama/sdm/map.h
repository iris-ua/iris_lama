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

// stl includes
#include <utility>
#include <unordered_map>

// local includes
#include "lama/types.h"
#include "lama/cow_ptr.h"
#include "lama/buffer_compressor.h"

#include "container.h"

namespace lama {

/**
 * Base class for any type of Map.
 *
 * A **Map** is a discrete (2D or 3D) representation of the environment.
 * Its unit is the **cell** that can represent any type of information. Because
 * the dimensions of a map is not known in dynamic applications (e.g. SLAM),
 * a map is sub-divided into smaller maps, called **patches**, that are only
 * allocated when a cell within its bounds is used.
 *
 */
class Map {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // The larger integer coordinate that a patch can have is ruled by the universal
    // constant, i.e. index < UNIVERSAL_CONSTANT. The universal constant bound can be calculated
    // by the cubic root of the maximum number that the patch index type can hold.
    // In this case, with a 64  bits index we have a universal constant equal to (2^64)^(1.0/3.0).
    // The constant is rounded to the nearest lower even number for practical reasons.
    static const uint64_t UNIVERSAL_CONSTANT = 2642244;

    // Use this magic numebr to identify a sparse-dense map (sdm) binary map.
    // The number is the hexadecimal encoding of '.smd'.
    static const uint32_t MAGIC = 0x6d64732e;

    // Version of the binary map supported by the library.
    static const uint16_t IO_VERSION = 0x0103;

    // Resolution of the map.
    double resolution;
    // The scale of the map, i.e. the inverse of the resolution.
    double scale;
    // The memory size of each individual cell
    const size_t cell_memory_size;

    // The length of the patch in cell units.
    uint32_t patch_length;
    // The number of cells in the patch volume.
    uint32_t patch_volume;

    // Less memory can be used when the map is used for 2d purposes.
    const bool is_3d;

    const uint32_t MASK3D;

    // IO header
    struct IOHeader {
        uint32_t magic;
        uint16_t version;
        uint32_t cell_size;
        uint32_t patch_length;
        size_t num_patches;
        float resolution;
        bool is_3d;
    };

    // Patches are kept on a sparse map and referenced by their unique id.
    // The container is wrapper around a copy-on-write structure so that we
    // share data efficiently, for example, duplicating a map only duplicates
    // patches that are accessed for writing during their lifetime.
    std::unordered_map<uint64_t, COWPtr< Container > > patches;

    virtual ~Map();

    /**
     * World to map coordinates.
     *
     * Convert continuous coordinates to discrete coordinates.
     * Because the coordinates are being discretized resolution is lost.
     * For example, in a map with 0.05 meters of resolution, the discrete
     * coordinates for (0, 0, 0) are the same for the coordinates (0.01, 0.02, 0.01).
     *
     * @param[in] coordinates Continuous coordinates to be converted.
     *
     * @returns The corresponding discrete coordinates.
     */
    inline Vector3ui w2m(const Vector3d& coordinates) const
    { return ((tf_ * coordinates).array() + 0.5).cast<uint32_t>(); }

    /**
     * World to map coordinates.
     *
     * Convert continuous coordinates to discrete coordinates without loosing the floating point.
     *
     * @param[in] coordinates Continuous coordinates to be converted.
     *
     * @returns The corresponding discrete coordinates.
     */
    inline Vector3d w2m_nocast(const Vector3d& coordinates) const
    { return tf_ * coordinates; }

    /**
     * Convert discrete coordinates to continuous coordinates.
     *
     * @param[in] coordinates Discrete coordinates to be converted.
     *
     * @returns The corresponding continuous coordinates.
     */
    inline Vector3d m2w(const Vector3ui& coordinates) const
    { return tf_inv_ * coordinates.cast<double>(); }

    /**
     * Convert discrete coordinates to patch index.
     */
    inline uint64_t m2p(const Vector3ui& coordinates) const
    {
        if (is_3d)
            return ((coordinates(0) >> log2dim) * UNIVERSAL_CONSTANT + (coordinates(1) >> log2dim)) * UNIVERSAL_CONSTANT +
                (coordinates(2) >> log2dim);
        else
            return (coordinates(0) >> log2dim) * UNIVERSAL_CONSTANT +
                (coordinates(1) >> log2dim);
    }

    /**
     * Convert patch index to discrete coordinates
     */
    inline Vector3ui p2m(uint64_t idx) const
    {
        if (is_3d){
            auto uc2 = UNIVERSAL_CONSTANT * UNIVERSAL_CONSTANT;
            return Vector3ui((idx / uc2) << log2dim,
                    ((idx % uc2) / UNIVERSAL_CONSTANT) << log2dim,
                    ((idx % uc2) % UNIVERSAL_CONSTANT) << log2dim);
        }
        // else
        return Vector3ui((idx / UNIVERSAL_CONSTANT) << log2dim,
                (idx % UNIVERSAL_CONSTANT) << log2dim, 0);
    }

    /**
     * Convert discrete coordinates to cell index.
     */
    inline uint32_t m2c(const Vector3ui& coord) const
    {
        const uint32_t mask = ((1<<log2dim)-1);

        return (coord(0) & mask) |
            ((coord(1) & mask) << log2dim) |
            ((coord(2) & mask & MASK3D) << (2*log2dim));
    }

    /**
     * Convert cell index to local grid discrete coordinates.
     */
    inline Vector3ui c2m(uint32_t idx) const
    {
        const uint32_t mask = ((1<<log2dim)-1);
        return { idx & mask, (idx >> log2dim) & mask, (idx >> (2*log2dim)) & mask & MASK3D };
    }

    /**
     * Get the size of allocated memory.
     *
     * It only counts the memory allocated for the patches.
     *
     * @returns The total size of memory used by the map.
     */
    size_t memory() const;
    size_t fullMemory() const;

    inline size_t numOfPatches() const
    { return patches.size(); }

    /**
     * Calculate the metric bounds of the map in map coordinates.
     *
     * @param[out] min The lowest coordinates in the map.
     * @param[out] max The highest coordinates in the map.
     */
    void bounds(Vector3ui& min, Vector3ui& max) const;

    inline void bounds(Vector3d& min, Vector3d& max) const
    {
        Vector3ui m, M; bounds(m, M);
        min = m2w(m); max = m2w(M);
    }

    /**
     * Verify if the map has a patch allocated at the given coordinates.
     *
     * @param[in] coordinates Check for a patch here.
     *
     * @returns true if a patch is allocated, false otherwise.
     */
    bool patchAllocated(const Vector3ui& coordinates) const;
    bool patchIsUnique(const Vector3ui& coordinates) const;

    void useCompression(bool compression, uint32_t lru_size = 50,
                        const std::string& algorithm = "lz4");

    uint64_t hash(const Vector3ui& coordinates) const;
    Vector3ui unhash(uint64_t idx, uint64_t stride = UNIVERSAL_CONSTANT) const;

    // Delete the patch that contains the given coordinates.
    bool deletePatchAt(const Vector3ui& coordinates);

    /**
     * Write map to a file.
     *
     * @param[in] filename Output filename.
     *
     * @returns true if successfull, false otherwise.
     */
    bool write(const std::string& filename) const;

    /**
     * Read map from a file.
     *
     * @param[in] filename Input filename.
     *
     * @returns true if successfull, false otherwise.
     */
    bool read(const std::string& filename);

    inline uint32_t cacheHit() const
    { return cache_hit_; }

    inline uint32_t cacheMiss() const
    { return cache_miss_; }

    using RayCallback = std::function<void(const Vector3ui&)>;

    // Compute ray and call user function for each cell the ray visits.
    void computeRay(const Vector3ui& from, const Vector3ui& to, const RayCallback& callback);

    void computeRay(const Vector3ui& from, const Vector3ui& to, VectorVector3ui& sink);
    void computeRay(const Vector3d& from, const Vector3d& to, VectorVector3ui& sink);


    /// A cell walker is a function that is called with the
    /// map coordinates of an existing cell.
    typedef std::function<void(const Vector3ui&)> CellWalker;

    /// The method is more simple and cleaner than implementing
    /// and using an iterator. For each cells it calls a walker.
    /// Lambdas are great for this.
    /* void visit_all_cells(const CellWalker& walker); */
    void visit_all_cells(const CellWalker& walker) const;

    /// A patch walker is a function that is called with the
    /// "origin" coordinates of an existing patch. The "origin"
    /// coordinates is in the global coordinates.
    typedef std::function<void(const Vector3ui&)> PatchWalker;

    /// Visit all existing patches and call the walker (or visitor) function
    /// for each individual patch.
    void visit_all_patches(const PatchWalker& walker) const;

protected:

    /**
     * Default constructor.
     *
     * @param[in] resolution The granularity of the map.
     * @param[in] cell_size  The memory size of a single cell.
     * @param[in] patch_size The size of the faces of the patch.
     * @param[in] is3d       Should the map represent a 3D environment.
     */
    Map(double resolution, size_t cell_size, uint32_t patch_size, bool is3d);

    /**
     * Copy constructor.
     */
    Map(const Map& other);

    /**
     * Get a pointer to the cell located at @p coordinates.
     *
     * If the cell does not exist, i.e. a patch with a cell at @p coordinates
     * does not exist, a new patch is allocated. With this behavior, the map
     * can grow dynamically.
     *
     * @param[in] coordinates Location of the cells in map coordinates.
     *
     * @returns A pointer to the desired cell.
     */
    uint8_t* get(const Vector3ui& coordinates);

    /**
     * Get a constant pointer to the cell located at @p coordinates.
     *
     * If the cell does not exist, i.e. a patch with a cell at @p coordinates
     * does not exist, a new patch is allocated. With this behavior, the map
     * can grow dynamically.
     *
     * @param[in] coordinates Location of the cells in map coordinates.
     *
     * @returns A constant pointer to the desired cell.
     */
    const uint8_t* get(const Vector3ui& coordinates) const;


    /**
     * Write internal parameters of the map.
     */
    virtual void writeParameters(std::ofstream& ) const
    {}

    /**
     * Read internal parameters of the map.
     */
    virtual void readParameters(std::ifstream& )
    {}


private:

    bool lru_key_exists(uint64_t idx) const;
    void lru_put(uint64_t idx, COWPtr< Container >* container) const;

    COWPtr< Container >* lru_get(uint64_t idx) const;

private:

    int log2dim; // patch lenght in bits

    Affine3d tf_;
    Affine3d tf_inv_;

    // LRU stuff
    typedef COWPtr< Container >* lru_type_t;
    typedef std::pair<uint64_t, lru_type_t> key_value_pair_t;
    typedef LinkedList<key_value_pair_t>::iterator list_iterator_t;

    mutable uint64_t prev_idx_ = -1;
    mutable COWPtr<Container>* prev_patch_;

    mutable LinkedList<key_value_pair_t>          lru_items_list_;
    mutable Dictionary<uint64_t, list_iterator_t> lru_items_map_;

    char* buffer_;
    size_t lru_max_size_;
    bool use_compression_;
    mutable uint32_t cache_miss_;
    mutable uint32_t cache_hit_;

    BufferCompressor* bc_;
};

} /* lama */

