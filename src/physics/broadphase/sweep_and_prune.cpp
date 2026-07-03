//
//  sweep_and_prune.cpp
//  engine::physics / broadphase
//

#include "engine/physics/broadphase/sweep_and_prune.h"

#include <algorithm>

namespace engine::physics::broadphase {

void sweepAndPrune(std::span<const Aabb> aabbs, std::vector<Pair>& pairs) {
    pairs.clear();
    const uint32_t n = static_cast<uint32_t>(aabbs.size());
    if (n < 2) return;

    // Pick the sweep axis with the largest spread of AABB centers (fewest false overlaps).
    Vec3 mean(0);
    for (const Aabb& a : aabbs) mean += (a.min + a.max) * Real(0.5);
    mean /= static_cast<Real>(n);
    Vec3 var(0);
    for (const Aabb& a : aabbs) {
        const Vec3 d = (a.min + a.max) * Real(0.5) - mean;
        var += d * d;
    }
    const int axis = (var.x >= var.y && var.x >= var.z) ? 0 : (var.y >= var.z ? 1 : 2);

    std::vector<uint32_t> order(n);
    for (uint32_t i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](uint32_t a, uint32_t b) { return aabbs[a].min[axis] < aabbs[b].min[axis]; });

    // Sweep with an active list; drop intervals that have ended before the current start.
    std::vector<uint32_t> active;
    for (uint32_t k = 0; k < n; ++k) {
        const uint32_t ci = order[k];
        const Real lo = aabbs[ci].min[axis];
        active.erase(std::remove_if(active.begin(), active.end(),
                     [&](uint32_t x) { return aabbs[x].max[axis] < lo; }), active.end());
        for (uint32_t x : active)
            if (overlaps(aabbs[x], aabbs[ci]))
                pairs.emplace_back(std::min(x, ci), std::max(x, ci));
        active.push_back(ci);
    }

    std::sort(pairs.begin(), pairs.end());   // deterministic pair order
}

} // namespace engine::physics::broadphase
