//
//  convex.h
//  engine::physics / collision
//
//  Convex↔convex collision: runs GJK, and on overlap EPA, producing a single Contact (normal
//  oriented A→B, an approximate contact point, and the penetration as a negative separation).
//  The fallback path for any shape pair without an exact primitive test.
//

#pragma once

#include "engine/physics/collision/contact.h"
#include "engine/physics/collision/support.h"

namespace engine::physics::collide {

bool convexVsConvex(const SupportShape& a, const SupportShape& b, Contact& out);

} // namespace engine::physics::collide
