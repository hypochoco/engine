//
//  thread_pool.cpp
//  engine::core / threading
//

#include "engine/core/threading/thread_pool.h"

#include <algorithm>
#include <atomic>

namespace engine::core {

ThreadPool::ThreadPool(unsigned threadCount) {
    if (threadCount == 0) {
        threadCount = std::thread::hardware_concurrency();
        if (threadCount == 0) threadCount = 1;
    }
    workers_.reserve(threadCount);
    for (unsigned i = 0; i < threadCount; ++i)
        workers_.emplace_back([this] { workerLoop(); });
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    for (std::thread& t : workers_)
        if (t.joinable()) t.join();
}

void ThreadPool::workerLoop() {
    for (;;) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lk(mutex_);
            cv_.wait(lk, [this] { return stop_ || !tasks_.empty(); });
            if (stop_ && tasks_.empty()) return;
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        task();
    }
}

void ThreadPool::parallelFor(std::size_t count, const std::function<void(std::size_t)>& fn,
                             std::size_t grain) {
    if (count == 0) return;
    if (grain == 0) grain = 1;

    std::atomic<std::size_t> cursor{ 0 };
    auto work = [&] {
        for (;;) {
            const std::size_t i = cursor.fetch_add(grain, std::memory_order_relaxed);
            if (i >= count) break;
            const std::size_t end = std::min(i + grain, count);
            for (std::size_t k = i; k < end; ++k) fn(k);
        }
    };

    const unsigned n = workerCount();
    std::mutex doneMx;
    std::condition_variable doneCv;
    unsigned pending = n;   // guarded by doneMx

    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (unsigned w = 0; w < n; ++w) {
            tasks_.push([&] {
                work();
                std::lock_guard<std::mutex> dlk(doneMx);   // decrement + notify under the lock
                if (--pending == 0) doneCv.notify_one();   // so the waiter can't destroy them early
            });
        }
    }
    cv_.notify_all();

    work();   // the calling thread participates

    std::unique_lock<std::mutex> dlk(doneMx);
    doneCv.wait(dlk, [&] { return pending == 0; });
}

} // namespace engine::core
