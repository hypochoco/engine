//
//  body.h
//  engine::physics / dynamics
//
//  Rigid-body state + material + inertia helpers. All physical parameters are DATA (§14.6) so
//  they can later become differentiable inputs. Rotational quantities (orientation + angular
//  velocity + inverse inertia) are first-class so bodies can truly roll.
//

#pragma once

#include "engine/physics/types.h"

namespace engine::physics {

enum class BodyType { Static, Kinematic, Dynamic };

struct PhysicsMaterial {
    Real restitution = Real(0.2);
    Real friction    = Real(0.5);
    Real compliance  = Real(0);   // 0 = rigid; >0 = soft/compliant contact (diff.-friendly)
};

// Per-body dynamics state. The world stores these in packed, index-stable arrays (§14.5);
// this struct documents the fields and layout.
struct RigidBodyState {
    Vec3 position{0};
    Quat orientation{1, 0, 0, 0};   // (w,x,y,z) identity; body -> world
    Vec3 linearVelocity{0};
    Vec3 angularVelocity{0};        // world frame
    Real invMass = Real(1);         // 0 => infinite mass (static/kinematic)
    Mat3 invInertiaLocal{Real(0)};  // body-space inverse inertia; 0 => no rotation
};

// --- inertia helpers ---

inline Mat3 solidSphereInertia(Real mass, Real radius) {
    const Real i = Real(2) / Real(5) * mass * radius * radius;
    return Mat3(i);   // diagonal (i,i,i)
}

inline Mat3 solidSphereInvInertia(Real mass, Real radius) {
    const Real denom = Real(2) / Real(5) * mass * radius * radius;
    return (denom > kEpsilon) ? Mat3(Real(1) / denom) : Mat3(Real(0));
}

// Solid box inverse inertia (body-space, diagonal). `half` = half-extents.
inline Mat3 solidBoxInvInertia(Real mass, const Vec3& half) {
    const Vec3 h2 = half * half;
    const Real ix = Real(1) / Real(3) * mass * (h2.y + h2.z);
    const Real iy = Real(1) / Real(3) * mass * (h2.x + h2.z);
    const Real iz = Real(1) / Real(3) * mass * (h2.x + h2.y);
    Mat3 inv(Real(0));
    inv[0][0] = ix > kEpsilon ? Real(1) / ix : Real(0);
    inv[1][1] = iy > kEpsilon ? Real(1) / iy : Real(0);
    inv[2][2] = iz > kEpsilon ? Real(1) / iz : Real(0);
    return inv;
}

// Capsule (local +Y axis) inverse inertia, cylinder approximation (caps' distribution ignored).
inline Mat3 solidCapsuleInvInertia(Real mass, Real radius, Real halfHeight) {
    const Real h = Real(2) * halfHeight;
    const Real iy = Real(0.5) * mass * radius * radius;                          // about the axis
    const Real ip = Real(1) / Real(12) * mass * (Real(3) * radius * radius + h * h);   // perpendicular
    Mat3 inv(Real(0));
    inv[0][0] = ip > kEpsilon ? Real(1) / ip : Real(0);
    inv[1][1] = iy > kEpsilon ? Real(1) / iy : Real(0);
    inv[2][2] = ip > kEpsilon ? Real(1) / ip : Real(0);
    return inv;
}

// World-space inverse inertia for the current orientation: R · I⁻¹_local · Rᵀ.
inline Mat3 worldInvInertia(const Quat& orientation, const Mat3& invInertiaLocal) {
    const Mat3 R = glm::mat3_cast(orientation);
    return R * invInertiaLocal * glm::transpose(R);
}

} // namespace engine::physics
