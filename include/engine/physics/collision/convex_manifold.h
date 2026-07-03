//
//  convex_manifold.h
//  engine::physics / collision
//
//  Generic contact manifold for two convex POLYTOPES (box/hull) given their world-space
//  vertices and the EPA result. Extracts the supporting face of each along the normal, then
//  clips the incident face against the reference face → up to 4 contact points (stable
//  stacking for hulls, mirroring what box_box does for boxes). Falls back to the single EPA
//  point when a genuine face pair isn't found (edge/vertex contact).
//

#pragma once

#include <span>

#include "engine/physics/collision/contact.h"
#include "engine/physics/types.h"

namespace engine::physics::collide {

int polytopeManifold(std::span<const Vec3> vertsA, std::span<const Vec3> vertsB,
                     const Contact& epa, Contact out[4]);

} // namespace engine::physics::collide
