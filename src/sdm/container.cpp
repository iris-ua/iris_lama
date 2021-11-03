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

#include <fstream>

#include "lama/print.h"
#include "lama/sdm/container.h"

lama::Container::Container(uint32_t log2dim)
    : mask(log2dim)
{ }

lama::Container::Container(const Container& other)
    : mask(other.mask)
{
    if (other.data == nullptr)
        return;

    memory_size  = other.memory_size;
    element_size = other.element_size;
    actual_memory_size = other.actual_memory_size;

    data = (uint8_t*) malloc(actual_memory_size);
    memcpy(data, other.data, actual_memory_size );
}

lama::Container::~Container()
{
    if (data != nullptr)
        free(data);
}

bool lama::Container::ok() const
{
    return (data != nullptr);
}

bool lama::Container::alloc(uint32_t size, uint32_t size_of_element)
{
    data = static_cast<uint8_t*>(calloc(size, size_of_element));
    if (data == nullptr)
        return false;

    memory_size  = size * size_of_element;
    element_size = size_of_element;
    actual_memory_size = memory_size;

    return true;
}

size_t lama::Container::memory() const
{
    return actual_memory_size;
}

size_t lama::Container::fullMemory() const
{
    return memory_size;
}

bool lama::Container::compress(BufferCompressor* bc, char* buffer)
{
    if (memory_size != actual_memory_size)
        return true; // already compressed

    char* new_data;
    auto compressed_size = bc->compress((const char*)data, memory_size, &new_data, buffer);

    if (compressed_size == 0)
        return false;

    free(data);
    data = (uint8_t*)new_data;
    actual_memory_size = compressed_size;

    return true;
}

bool lama::Container::decompress(BufferCompressor* bc)
{
    if (memory_size == actual_memory_size)
        return true; // already decompressed

    char* new_data;
    auto expected_size = bc->decompress((const char*)data, actual_memory_size, &new_data, memory_size);

    if (expected_size != memory_size)
        return false;

    free(data);
    data = (uint8_t*)new_data;
    actual_memory_size = memory_size;

    return true;
}

void lama::Container::write(BufferCompressor* bc, std::ofstream& stream) const
{
    uint8_t* write_data = data;

    // If the data is compressed we must descompress before writting.
    if (memory_size != actual_memory_size){
        char* new_data;
        auto expected_size = bc->decompress((const char*)data, actual_memory_size, &new_data, memory_size);

        if (expected_size != memory_size){
            // It failed to decompress. Lets write all zeros.
            write_data = (uint8_t*) calloc(1, memory_size);
        } else {
            write_data = (uint8_t*) new_data;
        }//end if
    }//end if

    stream.write((char*)write_data, memory_size);
    stream.write((char*)mask.mWords, sizeof(uint64_t) * mask.wordCount());

    if (write_data != data)
        free(write_data);
}

void lama::Container::read(std::ifstream& stream)
{
    // The container must be in its uncompressed state.
    if (memory_size != actual_memory_size){
        return;
    }

    stream.read((char*)data, memory_size);
    stream.read((char*)mask.mWords, sizeof(uint64_t)*mask.wordCount());
}

