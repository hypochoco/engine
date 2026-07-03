//
//  contact.h
//  engine::physics / collision
//
//  A single geometric contact. Deliberately SOLVER-AGNOSTIC (§14.4 of the physics plan): it
//  exposes a CONTINUOUS signed `separation` (negative = penetration depth, positive = gap),
//  the contact `normal`, and a world-space `point`, and bakes in no assumption about whether
//  the consumer is an impulse solver or a compliant/soft-force solver. Body identities are
//  attached later at the manifold level (Phase 1); Phase-0 primitive tests are purely
//  geometric.
//

#pragma once

#include "engine/physics/types.h"

namespace engine::physics {

struct Contact {
    Vec3 normal{0, 1, 0};   // unit; points from the first shape toward the second
    Vec3 point{0};          // world-space contact point
    Real separation = 0;    // signed distance between surfaces; < 0 when overlapping
    bool touching = false;  // separation <= margin
};

} // namespace engine::physics
