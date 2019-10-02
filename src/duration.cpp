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

#include <thread>

#include "lama/duration.h"

lama::Duration::Duration()
    : duration_(std::chrono::nanoseconds::zero())
{}

lama::Duration::Duration(const std::chrono::nanoseconds& nano)
    : duration_(nano)
{}

lama::Duration::Duration(double seconds)
    : duration_((int64_t)(seconds*1.e9))
{}

bool lama::Duration::isZero() const
{
    return duration_ == std::chrono::nanoseconds::zero();
}

bool lama::Duration::sleep() const
{
    std::this_thread::sleep_for(duration_);
    return true;
}

double lama::Duration::toSec() const
{
    double sec  = duration_.count()/1.e9;
    return sec;
}

int64_t lama::Duration::toNSec() const
{
    return duration_.count();
}

lama::Duration& lama::Duration::operator=(const Duration& rhs)
{
    duration_ = rhs.duration_;
    return *this;
}

bool lama::Duration::operator==(const Duration& rhs) const
{
    return (duration_ == rhs.duration_);
}

bool lama::Duration::operator!=(const Duration& rhs) const
{
    return (duration_ != rhs.duration_);
}

bool lama::Duration::operator<=(const Duration& rhs) const
{
    return (duration_ <= rhs.duration_);
}

bool lama::Duration::operator>=(const Duration& rhs) const
{
    return (duration_ >= rhs.duration_);
}

bool lama::Duration::operator<(const Duration& rhs) const
{
    return (duration_ < rhs.duration_);
}

bool lama::Duration::operator>(const Duration& rhs) const
{
    return (duration_ > rhs.duration_);
}

lama::Duration lama::Duration::operator+(const Duration& rhs) const
{
    return Duration(duration_ + rhs.duration_);
}

lama::Duration lama::Duration::operator-(const Duration& rhs) const
{
    return Duration(duration_ - rhs.duration_);
}

lama::Duration lama::Duration::operator-() const
{
    return Duration(-duration_);
}

lama::Duration& lama::Duration::operator+=(const Duration& rhs)
{
    duration_ += rhs.duration_;
    return *this;
}

lama::Duration& lama::Duration::operator-=(const Duration& rhs)
{
    duration_ -= rhs.duration_;
    return *this;
}

