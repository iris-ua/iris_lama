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
#include <mutex>

namespace lama {

// https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Copy-on-write
template <class T>
class COWPtr {
public:
    typedef std::shared_ptr<T> Ptr;

public:

    COWPtr(T* ptr = 0) : refptr_(ptr)
    {}

    COWPtr(Ptr& refptr) : refptr_(refptr)
    {}

    COWPtr(const COWPtr<T>& other)
    {
        refptr_ = other.refptr_;
    }

    inline COWPtr<T>& operator=(const COWPtr<T>& other)
    {
        refptr_ = other.refptr_;
        return *this;
    }

    inline T* get()
    {
        return refptr_.get();
    }

    inline const T* get() const
    {
        return refptr_.get();
    }

    inline bool unique() const
    {
        return refptr_.unique();
    }

    inline long use_count() const
    {
        return refptr_.use_count();
    }

    inline const T* operator->() const
    {
        return refptr_.operator->();
    }

    inline const T* read_only() const
    {
        return refptr_.operator->();
    }

    inline T* operator->()
    {
        detach();
        return refptr_.operator->();
    }

private:

    void detach()
    {
        if (refptr_.unique())
            return;

        T* tmp = refptr_.get();

        std::unique_lock<std::mutex> lock(mutex_);
        if (not refptr_.unique())
            refptr_ = Ptr( new T( *tmp ) );
    }

private:
    Ptr refptr_;
    std::mutex mutex_;
};

} /* lama */

