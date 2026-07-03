//
//  so3.cpp
//  engine::tst / physics / unit
//
//  Rotation math (SO(3) exp/log, orientation integration) and inertia helpers. The exp/log maps
//  have small-angle Taylor branches and a near-π atan2 branch; inertia helpers must match the
//  textbook tensors, and worldInvInertia must correctly rotate an anisotropic tensor.
//

#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/physics/dynamics/body.h"
#include "engine/physics/dynamics/integrate.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
bool approx(Real a, Real b, Real eps = Real(1e-4)) { return std::fabs(a - b) <= eps; }
bool approxVec(const Vec3& a, const Vec3& b, Real eps = Real(1e-4)) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}
} // namespace

// exp/log round-trips across the small-angle, mid, and near-π branches (all |φ| < π so the map
// is injective and log recovers exactly the same rotation vector).
TST_CASE(physics, unit, so3_roundtrip) {
    const Vec3 axis = glm::normalize(Vec3(0.3f, -0.6f, 0.7f));
    for (Real angle : { Real(0), Real(1e-7), Real(1e-3), Real(1.0), Real(2.5), Real(3.1) }) {
        const Vec3 phi = axis * angle;
        const Quat q = so3ExpMap(phi);
        TST_REQUIRE(approx(glm::length(q), 1.0f));        // always unit
        TST_REQUIRE(approxVec(so3LogMap(q), phi, 1e-4f)); // recovered
    }
}

// A full 2π spin about +y returns to the identity orientation, staying unit throughout.
TST_CASE(physics, unit, so3_full_spin) {
    const int N = 1000;
    const Vec3 omega(0, Real(2) * Real(M_PI), 0);         // 2π rad over unit time
    Quat q(1, 0, 0, 0);
    for (int i = 0; i < N; ++i) {
        q = integrateOrientation(q, omega, Real(1) / N);
        TST_REQUIRE(approx(glm::length(q), 1.0f));
    }
    TST_REQUIRE(approxVec(q * Vec3(0, 0, 1), Vec3(0, 0, 1), 1e-3f));  // back where it started
}

// Solid-box inverse inertia matches (1/3)m(hj²+hk²) per axis, including the anisotropic case.
TST_CASE(physics, unit, inertia_box) {
    // Cube: m=12, half=1 -> I = 1/3·12·(1+1) = 8 on every axis -> inv 0.125.
    Mat3 inv = solidBoxInvInertia(12.0f, Vec3(1, 1, 1));
    TST_REQUIRE(approx(inv[0][0], 0.125f) && approx(inv[1][1], 0.125f) && approx(inv[2][2], 0.125f));

    // Anisotropic: m=3, half=(1,2,3). Ix=1·(4+9)=13, Iy=1·(1+9)=10, Iz=1·(1+4)=5.
    inv = solidBoxInvInertia(3.0f, Vec3(1, 2, 3));
    TST_REQUIRE(approx(inv[0][0], 1.0f / 13.0f));
    TST_REQUIRE(approx(inv[1][1], 1.0f / 10.0f));
    TST_REQUIRE(approx(inv[2][2], 1.0f / 5.0f));
}

// Capsule (cylinder approx) inverse inertia: Iy = ½mr² (about axis), Iperp = 1/12·m(3r²+h²).
TST_CASE(physics, unit, inertia_capsule) {
    const Real m = 2, r = 0.5f, hh = 1.0f, h = 2 * hh;
    const Mat3 inv = solidCapsuleInvInertia(m, r, hh);
    const Real iy = 0.5f * m * r * r;                      // 0.25
    const Real ip = (1.0f / 12.0f) * m * (3 * r * r + h * h);  // 0.79166
    TST_REQUIRE(approx(inv[1][1], 1.0f / iy));
    TST_REQUIRE(approx(inv[0][0], 1.0f / ip));
    TST_REQUIRE(approx(inv[2][2], 1.0f / ip));
}

// worldInvInertia rotates the tensor: a 90° turn about z swaps the x and y diagonal entries.
TST_CASE(physics, unit, world_inv_inertia) {
    Mat3 local(0);
    local[0][0] = 2; local[1][1] = 5; local[2][2] = 9;    // distinct principal axes
    const Quat q = glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1));
    const Mat3 w = worldInvInertia(q, local);
    TST_REQUIRE(approx(w[0][0], 5.0f, 1e-4f));            // x now carries old-y value
    TST_REQUIRE(approx(w[1][1], 2.0f, 1e-4f));            // y now carries old-x value
    TST_REQUIRE(approx(w[2][2], 9.0f, 1e-4f));            // z unchanged
    TST_REQUIRE(std::fabs(w[0][1]) < 1e-4f && std::fabs(w[1][0]) < 1e-4f);
}
