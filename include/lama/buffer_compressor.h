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

#include "types.h"

namespace lama {

/**
 * Abstract in-memory buffer compressor.
 */
struct BufferCompressor {

    virtual ~BufferCompressor(){}

    /**
     * Compress a memory buffer.
     *
     * @param[in]  src       Memory buffer to be compressed.
     * @param[in]  src_size  The size in bytes of the uncompressed memory buffer.
     * @param[out] dst       Memory buffer that will hold the compressed data.
     *                       The buffer must _not_ be allocated, it will be allocated
     *                       by the function.
     * @param[in]  buffer    Intermediary buffer used during compression. Make
     *                       sure that its size is >= compressBound(@p src_size).
     *                       If buffer is *null* an internal one will be used.
     *
     * @return Number of bytes written into @p dst or 0 if the compression fails.
     */
    virtual size_t compress(const char* src, size_t src_size, char** dst, char* buffer = 0) = 0;

    /**
     * Decompress a memory buffer.
     */
    virtual size_t decompress(const char* src, size_t src_size, char** dst, size_t dst_size) = 0;

    /**
     * Calculate the maximum compressed size in worst case scenario.
     *
     * @param[in] size
     *
     * @return Maximum compressed size.
     */
    virtual size_t compressBound(size_t size) = 0;

    /**
     * Create a clone of the compressor.
     */
    virtual BufferCompressor* clone() const = 0;
};

struct LZ4BufferCompressor : public BufferCompressor {

    virtual ~LZ4BufferCompressor(){}

    size_t compress(const char* src, size_t src_size, char** dst, char* buffer = 0);
    size_t decompress(const char* src, size_t src_size, char** dst, size_t dst_size);
    size_t compressBound(size_t size);

    BufferCompressor* clone() const;
};

struct ZSTDBufferCompressor : public BufferCompressor {

    virtual ~ZSTDBufferCompressor(){}

    size_t compress(const char* src, size_t src_size, char** dst, char* buffer = 0);
    size_t decompress(const char* src, size_t src_size, char** dst, size_t dst_size);

    size_t compressBound(size_t size);

    BufferCompressor* clone() const;
};

} /* lama */

