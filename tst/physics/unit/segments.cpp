//
//  segments.cpp
//  engine::tst / physics / unit
//
//  Segment-based collision (capsule cores) and degenerate-geometry safety. The closest-point
//  routines have several branches (parallel segments, perpendicular/crossing, zero-length or
//  coincident) — the parallel and coincident branches are where NaNs and wrong normals hide.
//

#include <cmath>
#include <cstdio>

#include <glm/gtc/quaternion.hpp>

#include "engine/physics/collision/capsule.h"
#include "engine/physics/collision/primitives.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
bool finiteVec(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
bool approx(Real a, Real b, Real eps = Real(1e-4)) { return std::fabs(a - b) <= eps; }
const Quat kUp(1, 0, 0, 0);   // capsule axis along local +Y
} // namespace

// Two parallel vertical capsules overlapping along their length: distance is the lateral gap.
TST_CASE(physics, unit, capsule_capsule_parallel) {
    const Capsule cap{ 0.3f, 1.0f };
    Contact c;
    TST_REQUIRE(collide::capsuleVsCapsule(Vec3(0, 0, 0), kUp, cap, Vec3(0.5f, 0, 0), kUp, cap, Real(0), c));
    TST_REQUIRE(approx(c.separation, 0.5f - 0.6f));       // lateral 0.5 minus (0.3+0.3)
    TST_REQUIRE(approx(std::fabs(c.normal.x), 1.0f, 1e-3f));
    TST_REQUIRE(finiteVec(c.normal) && finiteVec(c.point));
}

// Crossing capsules (A vertical, B horizontal above it): closest features are A's top cap and
// B's midriff; the normal is vertical.
TST_CASE(physics, unit, capsule_capsule_crossing) {
    const Capsule cap{ 0.3f, 0.4f };
    const Quat horiz = glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1));   // B axis along X
    Contact c;
    TST_REQUIRE(collide::capsuleVsCapsule(Vec3(0, 0, 0), kUp, cap, Vec3(0, 0.7f, 0), horiz, cap, Real(0), c));
    // A-top (0,0.4,0) to B (0,0.7,0): gap 0.3, minus radii 0.6 -> -0.3.
    TST_REQUIRE(approx(c.separation, -0.3f, 1e-3f));
    TST_REQUIRE(approx(c.normal.y, 1.0f, 1e-3f));
}

// Coincident capsules (same center + orientation): distance 0 -> must fall back to a safe unit
// normal with no NaNs, and report full overlap.
TST_CASE(physics, unit, capsule_capsule_degenerate) {
    const Capsule cap{ 0.3f, 0.5f };
    Contact c;
    const bool hit = collide::capsuleVsCapsule(Vec3(1, 2, 3), kUp, cap, Vec3(1, 2, 3), kUp, cap, Real(0), c);
    TST_REQUIRE(hit);
    TST_REQUIRE(finiteVec(c.normal) && finiteVec(c.point));
    TST_REQUIRE(approx(glm::length(c.normal), 1.0f));     // unit fallback, not NaN/zero
    TST_REQUIRE(c.separation < 0);                        // overlapping
}

// Coincident spheres: zero center distance must not divide-by-zero.
TST_CASE(physics, unit, sphere_coincident) {
    Contact c;
    const bool hit = collide::sphereVsSphere(Vec3(0), Sphere{ 0.5f }, Vec3(0), Sphere{ 0.5f }, Real(0), c);
    TST_REQUIRE(hit);
    TST_REQUIRE(finiteVec(c.normal) && finiteVec(c.point));
    TST_REQUIRE(approx(glm::length(c.normal), 1.0f));
    TST_REQUIRE(approx(c.separation, -1.0f));             // -(0.5 + 0.5)
}

// Capsule vs sphere off the *end cap*: closest feature is the endpoint, not the segment middle.
TST_CASE(physics, unit, capsule_sphere_endcap) {
    const Capsule cap{ 0.3f, 0.5f };                      // endpoints at y = ±0.5
    Contact c;
    // Sphere beyond the top cap, straight up.
    TST_REQUIRE(collide::capsuleVsSphere(Vec3(0), kUp, cap, Vec3(0, 1.1f, 0), Sphere{ 0.3f }, Real(0), c));
    // top endpoint (0,0.5,0) to sphere (0,1.1,0): gap 0.6, minus radii 0.6 -> 0. Touching.
    TST_REQUIRE(approx(c.separation, 0.0f, 1e-3f));
    TST_REQUIRE(approx(c.normal.y, 1.0f, 1e-3f));
}
