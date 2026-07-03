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

// World-space inverse inertia for the current orientation: R · I⁻¹_local · Rᵀ.
inline Mat3 worldInvInertia(const Quat& orientation, const Mat3& invInertiaLocal) {
    const Mat3 R = glm::mat3_cast(orientation);
    return R * invInertiaLocal * glm::transpose(R);
}

} // namespace engine::physics
