//
//  parallel_sort.h
//  engine::core / threading
//
//  Parallel merge sort over a contiguous vector using a ThreadPool: sort ~P chunks in parallel,
//  then pairwise-merge the runs. Falls back to std::sort for small inputs or a single worker.
//  Must not be called from within a pool task (would nest parallelFor and starve workers).
//

#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <vector>

#include "engine/core/threading/thread_pool.h"

namespace engine::core {

template <class T, class Less = std::less<T>>
void parallelSort(ThreadPool& pool, std::vector<T>& v, Less less = Less{}) {
    const std::size_t n = v.size();
    const unsigned P = pool.workerCount() + 1;             // workers + the caller
    if (n < (std::size_t{1} << 15) || P <= 1) {            // small input → serial
        std::sort(v.begin(), v.end(), less);
        return;
    }

    const std::size_t chunk = (n + P - 1) / P;
    pool.parallelFor(P, [&](std::size_t i) {
        const std::size_t b = i * chunk;
        const std::size_t e = std::min(b + chunk, n);
        if (b < e) std::sort(v.begin() + b, v.begin() + e, less);
    }, 1);

    // Pairwise-merge sorted runs of length `sz` (the merge is serial but O(n) per level).
    for (std::size_t sz = chunk; sz < n; sz *= 2)
        for (std::size_t b = 0; b + sz < n; b += 2 * sz) {
            const std::size_t mid = b + sz;
            const std::size_t e = std::min(b + 2 * sz, n);
            std::inplace_merge(v.begin() + b, v.begin() + mid, v.begin() + e, less);
        }
}

} // namespace engine::core
