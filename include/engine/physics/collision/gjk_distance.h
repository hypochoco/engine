//
//  gjk_distance.h
//  engine::physics / collision
//
//  GJK distance query: the closest distance between two NON-overlapping convex shapes, plus the
//  witness points on each. Returns 0 when they overlap (caller falls back to EPA). Enables
//  accurate capsule↔convex contact (treat the capsule's core segment as a zero-radius shape,
//  then subtract the radius) instead of EPA's curved-surface approximation.
//

#pragma once

#include "engine/physics/collision/support.h"

namespace engine::physics {

// Returns the distance between a and b; fills witnessA/witnessB (closest points). 0 ⇒ overlap.
Real gjkClosest(const SupportShape& a, const SupportShape& b, Vec3& witnessA, Vec3& witnessB);

} // namespace engine::physics
