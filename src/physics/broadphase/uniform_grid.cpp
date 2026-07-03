//
//  uniform_grid.cpp
//  engine::physics / broadphase
//
//  Flat "index sort" uniform grid: instead of a hash map of per-cell vectors, we build a flat
//  array of (cellHash, bodyIndex) entries (one per covered cell), sort it, and pair bodies
//  within each equal-hash run. All contiguous memory + two sorts — much better cache behavior
//  than node-based containers, and the scratch is thread_local so repeated steps (and parallel
//  worlds on different threads) don't reallocate.
//

#include "engine/physics/broadphase/uniform_grid.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "engine/core/threading/parallel_sort.h"

namespace engine::physics::broadphase {
namespace {

// Teschner et al. spatial hash, folded to 32 bits (collisions only add candidates, never drop
// a true pair: overlapping AABBs always share a real cell → same hash → same run).
inline uint32_t cellHash(int32_t x, int32_t y, int32_t z) {
    return (static_cast<uint32_t>(x) * 73856093u)
         ^ (static_cast<uint32_t>(y) * 19349663u)
         ^ (static_cast<uint32_t>(z) * 83492791u);
}

} // namespace

void uniformGrid(std::span<const Aabb> aabbs, std::vector<Pair>& pairs, core::ThreadPool* pool) {
    pairs.clear();
    const uint32_t n = static_cast<uint32_t>(aabbs.size());
    if (n < 2) return;

    Real cell = Real(0);
    for (const Aabb& a : aabbs) {
        const Vec3 e = a.max - a.min;
        cell = std::max(cell, std::max(e.x, std::max(e.y, e.z)));
    }
    if (cell < kEpsilon) cell = Real(1);
    const Real inv = Real(1) / cell;
    auto coord = [&](Real v) { return static_cast<int32_t>(std::floor(v * inv)); };

    // entry = (cellHash << 32) | bodyIndex  → sorting groups by cell, then by body index.
    thread_local std::vector<uint64_t> entries;
    entries.clear();
    for (uint32_t i = 0; i < n; ++i) {
        const Aabb& a = aabbs[i];
        for (int32_t x = coord(a.min.x); x <= coord(a.max.x); ++x)
            for (int32_t y = coord(a.min.y); y <= coord(a.max.y); ++y)
                for (int32_t z = coord(a.min.z); z <= coord(a.max.z); ++z)
                    entries.push_back((static_cast<uint64_t>(cellHash(x, y, z)) << 32) | i);
    }
    if (pool) core::parallelSort(*pool, entries);
    else      std::sort(entries.begin(), entries.end());

    const size_t m = entries.size();
    for (size_t s = 0; s < m;) {
        const uint32_t h = static_cast<uint32_t>(entries[s] >> 32);
        size_t e = s + 1;
        while (e < m && static_cast<uint32_t>(entries[e] >> 32) == h) ++e;
        // Pair every distinct body in this cell run (indices already ascending).
        for (size_t a = s; a < e; ++a)
            for (size_t b = a + 1; b < e; ++b) {
                const uint32_t i = static_cast<uint32_t>(entries[a] & 0xffffffffu);
                const uint32_t j = static_cast<uint32_t>(entries[b] & 0xffffffffu);
                if (i != j && overlaps(aabbs[i], aabbs[j]))
                    pairs.emplace_back(std::min(i, j), std::max(i, j));
            }
        s = e;
    }

    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());   // pairs can share cells
}

} // namespace engine::physics::broadphase
