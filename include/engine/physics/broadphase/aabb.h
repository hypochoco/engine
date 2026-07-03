//
//  aabb.h
//  engine::physics / broadphase
//
//  Axis-aligned bounding box + overlap test, used by the broadphase to find candidate
//  colliding pairs cheaply before the exact narrowphase runs.
//

#pragma once

#include <cstdint>
#include <utility>

#include "engine/physics/types.h"

namespace engine::physics {

struct Aabb {
    Vec3 min{0};
    Vec3 max{0};

    static Aabb fromSphere(const Vec3& center, Real radius) {
        return { center - Vec3(radius), center + Vec3(radius) };
    }
    void expand(Real m) { min -= Vec3(m); max += Vec3(m); }
};

inline bool overlaps(const Aabb& a, const Aabb& b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x
        && a.min.y <= b.max.y && a.max.y >= b.min.y
        && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

namespace broadphase {
using Pair = std::pair<uint32_t, uint32_t>;   // indices into an aabb array, i < j
}

} // namespace engine::physics
