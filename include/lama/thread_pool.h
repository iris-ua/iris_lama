//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-09-26
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "types.h"

namespace lama {

struct TaskQueue;

struct ThreadPool {

    // We need to keep a track on our thread workers so that
    // we can join (wait) before finishing.
    List<std::thread> workers;

    // Tasks as submited to the queue by the user an fetched
    // by the workers. All concurrency problems are handled
    // by the queue.
    TaskQueue* queue;

    // Any working threads will exit, after finishing its task,
    // if stop is true.
    std::atomic<bool> stop {false};

    std::atomic<int> tasks_to_complete{0};

    std::mutex              queue_mutex;
    std::condition_variable queue_condition;

    std::mutex              wait_mutex;
    std::condition_variable wait_condition;

    // Keep track of hown many workers are at sleep waiting
    // for something to do.
    std::atomic<int> sleep_count;

    ThreadPool() = default;
    virtual ~ThreadPool();

    void init(size_t size = 0);

    // Enqueue (or post) a function to be executed by one
    // of the workers.
    void enqueue(std::function<void()>&& function);

    bool dequeue_task();

    // Wait until all tasks are completed.
    void wait();
};

} /* lama */

