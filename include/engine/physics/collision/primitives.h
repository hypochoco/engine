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

// Sphere vs oriented box (analytic closest-point; exact, unlike EPA on a curved shape).
// Contact normal points from the box toward the sphere (A = box, B = sphere convention).
bool sphereVsBox(const Vec3& boxCenter, const Quat& boxOrient, const Box& box,
                 const Vec3& sphereCenter, const Sphere& sphere, Real margin, Contact& out);

// Oriented box vs half-space plane: up to 4 deepest penetrating corners (a stable manifold for
// resting). Contact normal = plane normal (plane -> box). Returns the number of contacts.
int boxVsPlane(const Vec3& boxCenter, const Quat& boxOrient, const Box& box,
               const Plane& plane, Real margin, Contact out[4]);

// Generic: up to 4 deepest world-space points penetrating a half-space plane (for convex
// hulls). Contact normal = plane normal. Returns the number of contacts.
int pointsVsPlane(const Vec3* worldPoints, int count, const Plane& plane, Real margin, Contact out[4]);

} // namespace engine::physics::collide
