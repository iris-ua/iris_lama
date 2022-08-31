/*
 * IRIS Localization and Mapping (LaMa)
 *
 * Copyright (c) 2022-today, Eurico Pedrosa, University of Aveiro - Portugal
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

#include <chrono>
#include <map>

#include "types.h"

namespace lama {

struct TimerInfo {
    uint64_t min   = -1;
    uint64_t max   = 0;
    uint64_t total = 0;
    uint64_t count = 0;

    double mean = 0.0;
    double var  = 0.0;

    inline void acc(uint64_t duration)
    {
        min = std::min(min, duration);
        max = std::max(max, duration);

        total += duration;
        count += 1;

        double n = count;
        double d = duration;

        // Welford's online algorithm
        auto delta = d - mean;
        mean += delta / n;
        var  += delta*(d - mean);
    }
};

struct TimerContext {
    std::map<std::string, TimerInfo> timers;

    static TimerInfo* get_timer_info(const std::string& name)
    {
        TimerContext& ctx = TimerContext::global();

        auto it = ctx.timers.find(name);
        if (it == ctx.timers.end())
            it = ctx.timers.insert({name, TimerInfo{}}).first;

        return &(it->second);
    }

    static TimerContext& global()
    {
        static TimerContext ctx;
        return ctx;
    }

};

struct ScopedTimer {

    std::string name;
    std::chrono::steady_clock::time_point start;

    inline ScopedTimer(const std::string& name)
        : name(std::move(name))
        , start(std::chrono::steady_clock::now())
    {}

    inline ~ScopedTimer()
    {
        auto end = std::chrono::steady_clock::now();
        uint64_t duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

        auto info = TimerContext::get_timer_info(name);
        info->acc(duration);
    }

};

}// namespace lama
