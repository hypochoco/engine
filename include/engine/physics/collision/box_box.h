//
//  box_box.h
//  engine::physics / collision
//
//  Box-box contact manifold: GJK/EPA gives the separating normal + depth, then reference/
//  incident face clipping (Sutherland-Hodgman) produces up to 4 contact points — a stable
//  manifold so boxes rest and STACK without wobbling (a single EPA point cannot resist tipping
//  torque). Falls back to the single EPA point if clipping degenerates.
//

#pragma once

#include "engine/physics/collision/contact.h"
#include "engine/physics/shapes/shapes.h"
#include "engine/physics/types.h"

namespace engine::physics::collide {

// Returns the contact count (0..4). All contacts share the normal, oriented A -> B.
int boxVsBox(const Vec3& centerA, const Quat& orientA, const Box& a,
             const Vec3& centerB, const Quat& orientB, const Box& b, Contact out[4]);

} // namespace engine::physics::collide
