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

#include <memory>

#include "types.h"

namespace lama {

/// Data structure for images. It it is modele to work with
/// stb's image manipulation functions (https//github.com/nothings/stb/).
/// Most of the comments are borrowed from stb's source code.
struct Image {

    /// An image is a rectangle of pixels stored from left-to-right, top-to-bottom.
    /// The data pointer points to the first top-left-most pixel.
    /// The data can only be retained by a single object.
    std::unique_ptr<uint8_t[]> data;

    /// Each pixel contains N channels of data stored interleaved
    /// with 8 bits per channel, with the following order:
    ///     1 = Grayscale, 2 = Grayscale + Alpha, 3 = RGB, 4 = RGBA
    uint32_t channels;

    /// An image has `width` pixels wide.
    uint32_t width;
    /// An image has `height` pixels tall.
    uint32_t height;

    /// Check the image is valid, or not.
    inline bool ok() const
    { return data != nullptr; }

    /// Allocate data for an image of width `w`, height `h` and channels `c`.
    inline void alloc(uint32_t w, uint32_t h, uint32_t c)
    {
        width = w; height = h; channels = c;
        data.reset(new uint8_t[width*height*channels]);
    }

    /// Fill all pixels (and channels) with the same value.
    inline void fill(uint8_t value)
    { std::memset(data.get(), value, width * height * channels); }

    /// Access to a pixel by u,v and channel.
    /// @Warning There is no validation of the input parameters.
    inline uint8_t& operator()(uint32_t u, uint32_t v, uint32_t c = 0)
    { return data[(u * channels + v * width*channels) + c]; }

};

} /* lama */

