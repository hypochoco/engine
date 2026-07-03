//
//  integrate.h
//  engine::physics / dynamics
//
//  Pure integration kernels (§14.1: free functions over explicit state, no hidden state, dt is
//  always an explicit argument §14.7). Orientation is advanced through the SO(3) exponential
//  map (§14.2) rather than "add-and-renormalize", which keeps it 2nd-order clean and
//  differentiable for the future ML backend.
//

#pragma once

#include <cmath>

#include "engine/physics/types.h"

namespace engine::physics {

// Exponential map so(3) -> SO(3): a rotation vector (axis * angle) -> unit quaternion.
inline Quat so3ExpMap(const Vec3& rotVec) {
    const Real theta2 = glm::dot(rotVec, rotVec);
    if (theta2 < Real(1e-10)) {
        // Small-angle Taylor: sin(θ/2)/θ ≈ 1/2 − θ²/48 ; cos(θ/2) ≈ 1 − θ²/8.
        const Real s = Real(0.5) - theta2 * (Real(1) / Real(48));
        return glm::normalize(Quat(Real(1) - theta2 * (Real(1) / Real(8)),
                                   rotVec.x * s, rotVec.y * s, rotVec.z * s));
    }
    const Real theta = std::sqrt(theta2);
    const Real half  = theta * Real(0.5);
    const Real s     = std::sin(half) / theta;
    return Quat(std::cos(half), rotVec.x * s, rotVec.y * s, rotVec.z * s);
}

// Log map SO(3) -> so(3): unit quaternion -> rotation vector (axis * angle).
inline Vec3 so3LogMap(const Quat& q) {
    const Quat qq = (q.w < Real(0)) ? Quat(-q.w, -q.x, -q.y, -q.z) : q;  // shortest arc
    const Vec3 v(qq.x, qq.y, qq.z);
    const Real vn = glm::length(v);
    if (vn < Real(1e-5)) return v * Real(2);                 // ≈ 2v for small angle
    const Real angle = Real(2) * std::atan2(vn, qq.w);
    return v * (angle / vn);
}

// Advance orientation by a WORLD-frame angular velocity over dt (exp-map, left-multiply).
inline Quat integrateOrientation(const Quat& q, const Vec3& omegaWorld, Real dt) {
    return glm::normalize(so3ExpMap(omegaWorld * dt) * q);
}

// Semi-implicit (symplectic) Euler for linear motion: velocity updated first, then position.
inline void integrateLinear(Vec3& position, Vec3& velocity, const Vec3& accel, Real dt) {
    velocity += accel * dt;
    position += velocity * dt;
}

} // namespace engine::physics
