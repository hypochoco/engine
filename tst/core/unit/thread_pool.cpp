//
//  thread_pool_test.cpp
//  engine::tst
//
//  Correctness of core::ThreadPool::parallelFor — every index is visited exactly once, with
//  no races on distinct indices, and empty ranges are a no-op.
//

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "engine/core/threading/thread_pool.h"
#include "harness/harness.h"

TST_CASE(core, unit, thread_pool) {
    engine::core::ThreadPool pool;
    std::printf("workers = %u\n", pool.workerCount());
    TST_REQUIRE(pool.workerCount() >= 1);

    constexpr std::size_t N = 1'000'000;

    // Each index writes its own slot — verifies coverage + no cross-index races.
    std::vector<std::uint64_t> v(N, 0);
    pool.parallelFor(N, [&](std::size_t i) { v[i] = i + 1; }, 4096);
    std::uint64_t sum = 0;
    for (std::size_t i = 0; i < N; ++i) { TST_REQUIRE(v[i] == i + 1); sum += v[i]; }
    const std::uint64_t expected = static_cast<std::uint64_t>(N) * (N + 1) / 2;
    TST_REQUIRE(sum == expected);

    // Exactly N visits regardless of grain.
    std::atomic<std::size_t> visits{ 0 };
    pool.parallelFor(N, [&](std::size_t) { visits.fetch_add(1, std::memory_order_relaxed); }, 1000);
    TST_REQUIRE(visits.load() == N);

    // Empty range is a no-op (the body never runs).
    pool.parallelFor(0, [&](std::size_t) { TST_REQUIRE(false); });

    std::printf("thread pool ok (sum=%llu)\n", static_cast<unsigned long long>(sum));
}
