//
//  kernels.cpp
//  engine::tst / physics / unit
//
//  Backend-agnostic physics kernels + primitive collision + broadphase, checked against
//  closed-form / analytic results. No ECS, no graphics.
//

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/physics/broadphase/sweep_and_prune.h"
#include "engine/physics/broadphase/uniform_grid.h"
#include "engine/physics/physics.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
bool approx(Real a, Real b, Real eps = Real(1e-4)) { return std::fabs(a - b) <= eps; }
bool approxVec(const Vec3& a, const Vec3& b, Real eps = Real(1e-4)) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}
} // namespace

// Semi-implicit free fall matches the closed form: v = -g·dt·n, y = -g·dt²·n(n+1)/2.
TST_CASE(physics, unit, freefall) {
    const Real g = Real(9.81), dt = Real(0.01);
    const int n = 100;
    Vec3 pos(0), vel(0);
    for (int i = 0; i < n; ++i) integrateLinear(pos, vel, Vec3(0, -g, 0), dt);
    TST_REQUIRE(approx(vel.y, -g * dt * n));
    TST_REQUIRE(approx(pos.y, -g * dt * dt * (Real(n) * (n + 1) / Real(2))));
}

// SO(3) exp/log orientation integration (rotate +y by 90°) + round-trip.
TST_CASE(physics, unit, orientation) {
    const Real quarter = Real(M_PI) * Real(0.5);
    const Vec3 v(0, 0, 1);

    Quat q1 = integrateOrientation(Quat(1, 0, 0, 0), Vec3(0, quarter, 0), Real(1));
    TST_REQUIRE(approx(glm::length(q1), Real(1)));
    TST_REQUIRE(approxVec(q1 * v, Vec3(1, 0, 0), Real(1e-3)));

    Quat q = Quat(1, 0, 0, 0);
    for (int i = 0; i < 100; ++i) q = integrateOrientation(q, Vec3(0, quarter, 0), Real(1) / 100);
    TST_REQUIRE(approx(glm::length(q), Real(1)));
    TST_REQUIRE(approxVec(q * v, Vec3(1, 0, 0), Real(1e-3)));

    const Vec3 phi(0.2f, -0.5f, 0.3f);
    TST_REQUIRE(approxVec(phi, so3LogMap(so3ExpMap(phi)), Real(1e-4)));
}

// Solid-sphere inertia (2/5 m r²) and its inverse.
TST_CASE(physics, unit, inertia) {
    const Real m = Real(2), r = Real(0.5);
    const Mat3 I = solidSphereInertia(m, r);
    const Mat3 Iinv = solidSphereInvInertia(m, r);
    const Real expected = Real(2) / Real(5) * m * r * r;
    TST_REQUIRE(approx(I[0][0], expected));
    TST_REQUIRE(approx(Iinv[0][0], Real(1) / expected));
    TST_REQUIRE(approx((I * Iinv)[0][0], Real(1)));
}

// Exact sphere-plane and sphere-sphere contacts.
TST_CASE(physics, unit, sphere_contacts) {
    const Plane ground{ Vec3(0, 1, 0), Real(0) };
    const Sphere s{ Real(0.5) };
    Contact c;
    TST_REQUIRE(collide::sphereVsPlane(Vec3(0, 0.3f, 0), s, ground, Real(0), c));
    TST_REQUIRE(approx(c.separation, Real(-0.2)));
    TST_REQUIRE(approxVec(c.normal, Vec3(0, 1, 0)));
    TST_REQUIRE(approxVec(c.point, Vec3(0, -0.2f, 0)));
    Contact c2;
    TST_REQUIRE(!collide::sphereVsPlane(Vec3(0, 1.0f, 0), s, ground, Real(0), c2));
    TST_REQUIRE(approx(c2.separation, Real(0.5)));

    Contact cs;
    TST_REQUIRE(collide::sphereVsSphere(Vec3(0, 0, 0), Sphere{ 1 }, Vec3(1.5f, 0, 0), Sphere{ 1 }, Real(0), cs));
    TST_REQUIRE(approx(cs.separation, Real(-0.5)));
    TST_REQUIRE(approxVec(cs.normal, Vec3(1, 0, 0)));
}

// Both broadphases produce exactly the brute-force overlap set.
TST_CASE(physics, unit, broadphase) {
    std::vector<Aabb> boxes;
    for (int gx = 0; gx < 6; ++gx)
        for (int gy = 0; gy < 6; ++gy) {
            const Vec3 c(gx * 0.9f, gy * 0.9f, 0.0f);
            boxes.push_back(Aabb{ c - Vec3(0.5f), c + Vec3(0.5f) });
        }
    std::vector<broadphase::Pair> sap, grid, brute;
    broadphase::sweepAndPrune(boxes, sap);
    broadphase::uniformGrid(boxes, grid);
    for (uint32_t i = 0; i < boxes.size(); ++i)
        for (uint32_t j = i + 1; j < boxes.size(); ++j)
            if (overlaps(boxes[i], boxes[j])) brute.emplace_back(i, j);
    std::sort(brute.begin(), brute.end());
    TST_REQUIRE(sap == brute);
    TST_REQUIRE(grid == brute);
    std::printf("broadphase: %zu pairs\n", brute.size());
}
