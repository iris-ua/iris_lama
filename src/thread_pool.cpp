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

#include <chrono>
#include <thread>

#include "moodycamel/concurrentqueue.h"
#include "lama/thread_pool.h"

namespace lama {

template <typename T>
using ConcurrentQueue = moodycamel::ConcurrentQueue<T>;

struct TaskQueue {
    using CQueue = ConcurrentQueue<std::function<void()>>;
    CQueue tasks;
};

} /* lama */

void lama::ThreadPool::init(size_t size)
{
    queue = new TaskQueue;

    if (size == 0) size = std::thread::hardware_concurrency();
    for (size_t i = 0; i < size; ++i)
        workers.emplace_back(std::thread([this]{
            while (not stop.load(std::memory_order_relaxed)){
                if (not dequeue_task()){
                    ++sleep_count;
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    queue_condition.wait_for(lock, std::chrono::milliseconds(32));
                    --sleep_count;
                }// end if
            }// end while
        }));
}

lama::ThreadPool::~ThreadPool()
{
    stop = true;
    queue_condition.notify_all();

    for (auto& thread : workers)
        thread.join();

    delete queue;
}

void lama::ThreadPool::enqueue(std::function<void()>&& function)
{
    ++tasks_to_complete;
    queue->tasks.enqueue(std::move(function));
    if (sleep_count > 0){
        queue_condition.notify_one();
    }
}

bool lama::ThreadPool::dequeue_task()
{
    std::function<void()> task;
    if (queue->tasks.try_dequeue(task)){
        task();

        std::unique_lock<std::mutex> lock(wait_mutex);
        --tasks_to_complete;

        if (not tasks_to_complete)
            wait_condition.notify_all();

        return true;
    }

    return false;
}

void lama::ThreadPool::wait()
{
    std::unique_lock<std::mutex> lock(wait_mutex);

    while (tasks_to_complete)
        wait_condition.wait(lock);
}

