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

#include <cstdarg>
#include <iostream>

#include "lama/print.h"

namespace  {

void fmt_appendV(std::string* dst, const char* format, va_list ap)
{
    char buffer[1024];

    // To prevent possible changes to *ap*, we use a copy.
    va_list cap;
    va_copy(cap, ap);
    int result = vsnprintf(buffer, sizeof(buffer), format, cap);
    va_end(cap);

    if (result < (int)sizeof(buffer)){
        if (result < 0)
            return; // vnsprintf failed :(

        dst->append(buffer, result);
        return;
    }

    // The buffer was too small. Lucky for us vnsprintf
    // tells us how much memory it needs.
    int len = result + 1; // +1 for '\0'
    char* heap_buffer = new char[len];

    va_copy(cap, ap);
    result = vsnprintf(heap_buffer, len, format, cap);
    va_end(cap);

    if (result >=0 && result < len)
        dst->append(heap_buffer, result);

    delete [] heap_buffer;
}

} // namespace

// Output a formated string to stdout.
void lama::print(const char* format, ...)
{
    std::string message;

    va_list  ap;
    va_start(ap, format);
    fmt_appendV(&message, format, ap);
    va_end(ap);

    std::cout << message;
}

// Create a string from a formated text.
std::string lama::format(const char* format, ...)
{
    std::string message;

    va_list  ap;
    va_start(ap, format);
    fmt_appendV(&message, format, ap);
    va_end(ap);

    return message;
}

