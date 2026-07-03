//
//  primitives.h
//  engine::physics / collision
//
//  Exact closed-form collision tests for the common shape pairs (the "fast paths" that bypass
//  GJK/EPA). Each fills a solver-agnostic Contact and returns whether the shapes are touching
//  (separation <= margin). Pure functions of explicit geometry (§14.1).
//

#pragma once

#include "engine/physics/collision/contact.h"
#include "engine/physics/shapes/shapes.h"
#include "engine/physics/types.h"

namespace engine::physics::collide {

// Sphere (center in world) vs plane half-space. Contact normal = plane normal (points from
// the plane toward the sphere); point is the sphere's lowest point along the normal.
bool sphereVsPlane(const Vec3& center, const Sphere& sphere,
                   const Plane& plane, Real margin, Contact& out);

// Sphere vs sphere. Contact normal points from A toward B.
bool sphereVsSphere(const Vec3& centerA, const Sphere& a,
                    const Vec3& centerB, const Sphere& b, Real margin, Contact& out);

} // namespace engine::physics::collide
