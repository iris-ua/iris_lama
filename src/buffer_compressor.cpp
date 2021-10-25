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

#include "lama/buffer_compressor.h"

#include "extern/lz4/lz4.h"
#include "extern/zstd/zstd.h"

size_t lama::LZ4BufferCompressor::compress(const char* src, size_t src_size, char** dst, char* buffer)
{
    const int max_destiny_size = LZ4_compressBound(src_size);
    bool free_buffer = true;

    if (buffer == 0)
        buffer = new char[max_destiny_size];
    else
        free_buffer = false;

    int compressed_size = LZ4_compress_fast(src, buffer, src_size, max_destiny_size, 1);

    if (compressed_size > 0){
        *dst = new char[compressed_size];
        memcpy(*dst, buffer, compressed_size);
    }

    if (free_buffer)
        delete [] buffer;

    return (size_t)compressed_size;
}

size_t lama::LZ4BufferCompressor::decompress(const char* src, size_t src_size, char** dst, size_t dst_size)
{
    *dst = new char[dst_size];

    int compressed_size = LZ4_decompress_fast(src, *dst, dst_size);

    if (compressed_size < 0){
        delete [] *dst;
        *dst = 0;
        return 0;
    }

    return dst_size;
}

size_t lama::LZ4BufferCompressor::compressBound(size_t size)
{
    return LZ4_compressBound(size);
}

lama::BufferCompressor* lama::LZ4BufferCompressor::clone() const
{
    return new LZ4BufferCompressor();
}

size_t lama::ZSTDBufferCompressor::compress(const char* src, size_t src_size, char** dst, char* buffer)
{
    const size_t max_destiny_size = ZSTD_compressBound(src_size);
    bool free_buffer = true;

    if (buffer == 0)
        buffer = new char[max_destiny_size];
    else
        free_buffer = false;

    int compressed_size = ZSTD_compress(buffer, max_destiny_size, src, src_size, 1);

    if (compressed_size > 0){
        *dst = new char[compressed_size];
        memcpy(*dst, buffer, compressed_size);
    }

    if (free_buffer)
        delete [] buffer;

    return (size_t)compressed_size;
}

size_t lama::ZSTDBufferCompressor::decompress(const char* src, size_t src_size, char** dst, size_t dst_size)
{
    *dst = new char[dst_size];

    int compressed_size = ZSTD_decompress(*dst, dst_size, src, src_size);

    if (compressed_size <= 0){
        delete [] *dst;
        *dst = 0;
        return 0;
    }

    return dst_size;
}

size_t lama::ZSTDBufferCompressor::compressBound(size_t size)
{
    return ZSTD_compressBound(size);
}

lama::BufferCompressor* lama::ZSTDBufferCompressor::clone() const
{
    return new ZSTDBufferCompressor();
}

