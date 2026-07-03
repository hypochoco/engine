//
//  epa.h
//  engine::physics / collision
//
//  Expanding Polytope Algorithm: given a GJK simplex enclosing the origin (overlap), expand
//  it toward the Minkowski-difference surface to recover the penetration normal + depth
//  (minimum translation to separate the two convex shapes).
//

#pragma once

#include "engine/physics/collision/gjk.h"
#include "engine/physics/collision/support.h"

namespace engine::physics {

struct EpaResult {
    Vec3 normal{0, 1, 0};   // unit; minimum-translation direction (orientation fixed by caller)
    Real depth = 0;         // penetration depth along `normal`
    bool ok = false;
};

EpaResult epaPenetration(const SupportShape& a, const SupportShape& b, const Simplex& simplex);

} // namespace engine::physics
