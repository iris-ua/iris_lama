/*
 * IRIS Localization and Mapping (LaMa)
 *
 * Copyright (c) 2020-today, Eurico Pedrosa, University of Aveiro - Portugal
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

#include <queue>
#include <chrono>
#include <thread> // for this_thread::sleep_for

#include "types.h"

namespace lama {

struct Duration {

    // Nanoseconds should be enough for most applications.
    std::chrono::nanoseconds ns;

    explicit Duration(double seconds)
        : ns((int64_t)(seconds*1.e9))
    {}

    Duration(const std::chrono::nanoseconds& nano)
        : ns(nano)
    {}

    Duration()
        : ns(std::chrono::nanoseconds::zero())
    {}

    inline Duration& operator=(const Duration& rhs)
    {
        ns = rhs.ns;
        return *this;
    }

    // Check for a zero duration.
    inline bool isZero() const
    { return ns == std::chrono::nanoseconds::zero(); }

    // sleep for the amount of time in `duration`
    inline void sleep() const
    { return std::this_thread::sleep_for(ns); }

    // Conversions
    inline double toSec() const
    { return ns.count() / 1.e9; }

    inline int64_t toNSec() const
    { return ns.count(); }

    // Logic with durations
    inline bool operator==(const Duration& rhs) const
    { return ns == rhs.ns; }

    inline bool operator!=(const Duration& rhs) const
    { return ns != rhs.ns; }

    inline bool operator<=(const Duration& rhs) const
    { return ns <= rhs.ns; }

    inline bool operator>=(const Duration& rhs) const
    { return ns <= rhs.ns; }

    inline bool operator<(const Duration& rhs) const
    { return ns < rhs.ns; }

    inline bool operator>(const Duration& rhs) const
    { return ns > rhs.ns ; }

    // Aritmetic operations with durations
    inline Duration operator+(const Duration& rhs) const
    { return Duration(ns + rhs.ns ); }

    inline Duration operator-(const Duration& rhs) const
    { return Duration(ns - rhs.ns ); }

    inline Duration operator*(double scale) const
    { return Duration(toSec()*scale); }

    inline Duration operator-() const
    { return Duration(-ns); }

    inline Duration& operator+=(const Duration& rhs)
    {
        ns += rhs.ns;
        return *this;
    }

    inline Duration& operator-=(const Duration& rhs)
    {
        ns -= rhs.ns;
        return *this;
    }
};

struct Time {

    Duration duration;

    // Get current wall time.
    static inline Time now()
    { return Time(Duration(std::chrono::system_clock::now().time_since_epoch())); }

    // Sleep until `end` time is reach.
    static inline void sleepUntil(const Time& end)
    {
        Duration d = end - Time::now();
        if (d > Duration(0))
            d.sleep();
    }

    // Time will be zero.
    Time() = default;

    Time(const Duration& duration)
        : duration(duration)
    {}

    explicit Time(double seconds)
        : duration(seconds)
    {}

    inline Time& operator=(const Time& rhs)
    {
        duration = rhs.duration;
        return *this;
    }

    // Check for zero time.
    inline bool isZero() const
    { return duration.isZero(); }

    // Conversions
    inline double toSec() const
    { return duration.toSec(); }

    inline int64_t toNSec() const
    { return duration.toNSec(); }

    // Logic with time
    inline bool operator==(const Time& rhs) const
    { return duration == rhs.duration; }

    inline bool operator!=(const Time& rhs) const
    { return duration != rhs.duration; }

    inline bool operator<=(const Time& rhs) const
    { return duration <= rhs.duration; }

    inline bool operator>=(const Time& rhs) const
    { return duration >= rhs.duration; }

    inline bool operator<(const Time& rhs) const
    { return duration > rhs.duration; }

    inline bool operator>(const Time& rhs) const
    { return duration > rhs.duration; }

    // The difference between times returns a duration in time.
    inline Duration operator-(const Time& rhs) const
    { return duration - rhs.duration; }

    // Operations with time durations
    inline Time operator+(const Duration& rhs) const
    {
        Time t = *this;
        t.duration += rhs;
        return t;
    }

    inline Time operator-(const Duration& rhs) const
    {
        Time t = *this;
        t.duration -= rhs;
        return t;
    }
};

struct Timer {
    // the timer started at this point in time.
    Time time_point;

    // Use immediately if you what to start the clock
    // as soon as the object is created.
    Timer(bool immediately = false)
    { if (immediately) start(); }

    inline void start()
    { time_point = Time::now(); }

    // Alias to start
    inline void reset()
    { start(); }

    inline Duration elapsed() const
    { return (Time::now() - time_point); }
};

struct Rate {

    Time start;
    Duration cycle;
    Duration actual_cycle;

    Rate(double frequency)
        : start(Time::now()),
          cycle(1.0 / frequency),
          actual_cycle(0.0)
    {}

    void sleep()
    {
        Time now = Time::now();
        Time end = (now < start ? now : start) + cycle;
        Duration span = end - now;

        actual_cycle = now - start;
        start = end;

        if (span <= Duration(0.0)){
            // reset if full cyle completed or time jump
            if (now > end + cycle)
                start = now;
            return;
        }
        span.sleep();
    }
};

struct EventFrequency {

    const uint32_t window;

    Timer timer;
    // event queue
    std::queue<Duration> queue;

    EventFrequency(uint32_t window_size = 30)
        : window(window_size)
    { timer.start(); }

    inline void reset()
    {
        timer.reset();
        queue = std::queue<Duration>();
    }

    // call this whenever an event happens.
    inline void event()
    {
        queue.push(timer.elapsed());
        if (queue.size() > window)
            queue.pop();
    }

    // call this with a timestamp to use simulated time.
    inline void event(const double timestamp)
    {
        queue.push(Duration(timestamp));
        if (queue.size() > window)
            queue.pop();
    }

    inline double getFrequency() const
    {
        if (queue.size() < 2)
            return 0.0;
        return (queue.size() - 1) / (queue.back() - queue.front()).toSec();
    }

};

} // lama
