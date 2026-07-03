//
//  thread_pool.h
//  engine::core / threading
//
//  A small fixed-size worker pool with a blocking parallelFor. Reused across subsystems (first
//  consumer: stepping many independent physics worlds in parallel — the milestone's "parallel
//  simulations" / ML many-envs throughput lever). The calling thread also participates in
//  parallelFor, so all cores are used. Not for nested parallelFor (would starve workers).
//

#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace engine::core {

class ThreadPool {
public:
    // threadCount == 0 -> hardware_concurrency worker threads.
    explicit ThreadPool(unsigned threadCount = 0);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    unsigned workerCount() const { return static_cast<unsigned>(workers_.size()); }

    // Invokes fn(i) for each i in [0, count), distributed dynamically across workers + the
    // caller. Blocks until every index has completed. `grain` = indices claimed per steal.
    void parallelFor(std::size_t count, const std::function<void(std::size_t)>& fn,
                     std::size_t grain = 1);

private:
    void workerLoop();

    std::vector<std::thread>          workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex                        mutex_;
    std::condition_variable           cv_;
    bool                              stop_ = false;
};

} // namespace engine::core
