//
//  capsule.h
//  engine::physics / collision
//
//  Analytic capsule collision (capsule = a segment swept by a radius). Segment-based closest-
//  point tests are exact and robust — better than EPA, which only approximates the capsule's
//  curved surface. Capsule vs box/hull still falls back to GJK/EPA (see the backend).
//

#pragma once

#include <glm/gtc/quaternion.hpp>

#include "engine/physics/collision/contact.h"
#include "engine/physics/collision/support.h"
#include "engine/physics/shapes/shapes.h"
#include "engine/physics/types.h"

namespace engine::physics::collide {

// World-space endpoints of a capsule's core segment.
inline void capsuleSegment(const Vec3& center, const Quat& orient, const Capsule& cap,
                           Vec3& a, Vec3& b) {
    const Vec3 axis = orient * Vec3(0, cap.halfHeight, 0);
    a = center - axis;
    b = center + axis;
}

// Normal points from A toward B.
bool capsuleVsSphere(const Vec3& capCenter, const Quat& capOrient, const Capsule& cap,
                     const Vec3& sphereCenter, const Sphere& sphere, Real margin, Contact& out);

bool capsuleVsCapsule(const Vec3& cA, const Quat& qA, const Capsule& a,
                      const Vec3& cB, const Quat& qB, const Capsule& b, Real margin, Contact& out);

// Capsule vs half-space plane: up to 2 contacts (both endpoints when lying flat → stable rest).
// Normal = plane normal (plane -> capsule). Returns the contact count.
int capsuleVsPlane(const Vec3& capCenter, const Quat& capOrient, const Capsule& cap,
                   const Plane& plane, Real margin, Contact out[2]);

// Capsule vs a convex shape (box/hull) via GJK distance on the capsule's core segment: accurate
// normal + depth (unlike EPA on the capsule's curved surface). Up to 2 contacts (both endpoints
// when the capsule lies against a face). Normal points from the convex toward the capsule.
// Returns 0 when overlapping/no-contact (caller may fall back to EPA).
int capsuleVsConvex(const Vec3& capCenter, const Quat& capOrient, const Capsule& cap,
                    const SupportShape& convex, Real margin, Contact out[2]);

} // namespace engine::physics::collide
