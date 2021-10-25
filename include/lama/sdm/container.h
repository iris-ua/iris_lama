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

#include "lama/types.h"
#include "lama/buffer_compressor.h"
#include "lama/node_mask.h"

namespace lama {

// Data container that holds a fixed number of elements.
// The allocated data can be compressed/decompressed on request.
// In practice you can store any type of element (i.e. data stucture),
// however, keep in mind that nay constructors and/or destructors
// will never be called, the container works only with plain c structures.
// Furthermore, all allocated data are initiallized with zero.
struct Container {

    // The data is stored in a continuous chunk of memory.
    uint8_t* data = nullptr;
    // And we keep track of allocated data and element size.
    uint32_t memory_size  = 0;
    uint32_t element_size = 0;
    // But because data can be compressed we need its actual size;
    uint32_t actual_memory_size = 0;

    // Bitmask that keeps tracking of the cells that were access.
    // This allows for faster iteration of the "known" cells.
    Mask mask;

    Container(uint32_t log2dim);
    Container(const Container& other);

    virtual ~Container();

    /**
     * Verify if the container is already allocated.
     */
    bool ok() const;

    // Allocate memory for the container.
    // The allocated memory is set to zero. The `calloc` function is used
    // for this purpose. [Hopefully] copy-on-write is used to zero out the memory.
    // Return true on success.
    bool alloc(uint32_t size, uint32_t size_of_element);

    /**
     *
     */
    size_t memory() const;

    /**
     *
     */
    size_t fullMemory() const;

    /**
     * Access data by index.
     *
     * This is an unsafe function because it does not verify
     * if the container is already allocated and the index in within
     * bounds.
     *
     * @returns A pointer to the data.
     */
    //T* get(uint32_t idx);

    inline uint8_t* get(uint32_t idx)
    {
        if (!mask.isOn(idx)) mask.setOn(idx);
        return (data + idx * element_size);
    }

    /**
     * Access data by index (constant version).
     *
     * This is an unsafe function because it does not verify
     * if the container is already allocated and the index in within
     * bounds.
     *
     * @returns A constant pointer to the data.
     */
    //const T* get(uint32_t idx) const;

    inline const uint8_t* get(uint32_t idx) const
    {
        if (!mask.isOn(idx)) return nullptr;
        return (data + idx * element_size);
    }

    bool compress(BufferCompressor* bc, char* buffer);
    bool decompress(BufferCompressor* bc);

    inline bool isCompressed() const
    { return actual_memory_size != memory_size; }

    void write(BufferCompressor* bc, std::ofstream& stream) const;
    void read(std::ifstream& stream);
};

} /* lama */

