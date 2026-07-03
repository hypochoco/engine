//
//  shapes.h
//  engine::physics / shapes
//
//  Phase-0 collision shapes: sphere + plane (half-space). More convex shapes and the GJK
//  `support(dir)` seam arrive in Phase 2; these two cover the milestone and the 100k-sphere
//  case via the exact primitive tests in collision/primitives.h.
//

#pragma once

#include "engine/physics/types.h"

namespace engine::physics {

struct Sphere {
    Real radius = Real(0.5);
};

// Half-space plane: the surface is { x : dot(normal, x) = offset } with `normal` unit-length;
// the signed distance of a point p is dot(normal, p) - offset (positive on the normal side).
struct Plane {
    Vec3 normal{0, 1, 0};
    Real offset = Real(0);
};

} // namespace engine::physics
