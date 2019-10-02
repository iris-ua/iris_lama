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

#include <map>
#include <queue>

namespace lama {

// Priority queue for integer as priority.
// Based on bucketedqueue.h from Boris Lau
// (http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/)
template <typename T>
class PriorityQueue {
    typedef std::map<int, std::queue<T> > bucket;

    int count;
    bucket buckets;
    typename bucket::iterator next;

public:

    PriorityQueue()
    {
        next = buckets.end();
        count = 0;
    }

    virtual ~PriorityQueue(){}

    inline void clear() { buckets.clear(); }

    // Checks whether the Queue is empty
    inline bool empty()
    { return (count==0); }

    // Insert an element
    void push(int priority, T element)
    {
        buckets[priority].push(element);
        if (next == buckets.end() || priority < next->first)
            next = buckets.find(priority);

        count++;
    }

    // Return (and pop) the element with the lowest priority.
    T pop()
    {
        while (next != buckets.end() && next->second.empty())
            ++next;

        T element = next->second.front();
        next->second.pop();
        if (next->second.empty()) {
            typename bucket::iterator it = next;
            next++;
            buckets.erase(it);
        }
        count--;
        return element;
    }

    inline int size()
    { return count; }

    inline int getNumBuckets()
    { return buckets.size(); }

};

} /* lama */

