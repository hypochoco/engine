//
//  sphere_box.cpp
//  engine::tst / physics / unit
//
//  Edge cases for the analytic sphere-vs-oriented-box test: the sphere's closest feature can be
//  a face, edge, or corner of the box, or the sphere center can be *inside* the box (deepest-
//  axis push-out). Also checks the oriented (rotated-box) path. Normal convention: box -> sphere.
//

#include <cmath>
#include <cstdio>

#include <glm/gtc/quaternion.hpp>

#include "engine/physics/collision/primitives.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
bool approx(Real a, Real b, Real eps = Real(1e-4)) { return std::fabs(a - b) <= eps; }
bool approxVec(const Vec3& a, const Vec3& b, Real eps = Real(1e-4)) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}
const Quat kNoRot(1, 0, 0, 0);
const Box  kUnitBox{ Vec3(1, 1, 1) };   // half-extents 1
} // namespace

// Face: sphere approaches a face straight-on. Closest feature is the face; normal is axis-aligned.
TST_CASE(physics, unit, sphere_box_face) {
    Contact c;
    // Clear: center 2 away, radius 0.5 -> gap 0.5.
    TST_REQUIRE(!collide::sphereVsBox(Vec3(0), kNoRot, kUnitBox, Vec3(2, 0, 0), Sphere{ 0.5f }, Real(0), c));
    TST_REQUIRE(approx(c.separation, 0.5f));
    TST_REQUIRE(approxVec(c.normal, Vec3(1, 0, 0)));
    TST_REQUIRE(approxVec(c.point, Vec3(1, 0, 0)));      // closest point on the box face
    // Penetrating: center 1.4 away -> separation -0.1.
    TST_REQUIRE(collide::sphereVsBox(Vec3(0), kNoRot, kUnitBox, Vec3(1.4f, 0, 0), Sphere{ 0.5f }, Real(0), c));
    TST_REQUIRE(approx(c.separation, -0.1f));
    TST_REQUIRE(approxVec(c.normal, Vec3(1, 0, 0)));
}

// Edge: sphere sits off a box edge (two coords past the extent). Normal is diagonal in that plane.
TST_CASE(physics, unit, sphere_box_edge) {
    Contact c;
    const Real d = 0.30f;                                // offset past the +x/+y edge
    const Vec3 center(1 + d, 1 + d, 0);
    const Real expectedSep = std::sqrt(2 * d * d) - 0.5f; // dist from edge minus radius
    TST_REQUIRE(collide::sphereVsBox(Vec3(0), kNoRot, kUnitBox, center, Sphere{ 0.5f }, Real(0), c) == (expectedSep <= 0));
    TST_REQUIRE(approx(c.separation, expectedSep));
    TST_REQUIRE(approxVec(c.normal, Vec3(0.70710678f, 0.70710678f, 0)));
    TST_REQUIRE(approxVec(c.point, Vec3(1, 1, 0)));       // the edge line at z=0
}

// Corner: sphere off a box corner (three coords past the extent). Normal is the (1,1,1) diagonal.
TST_CASE(physics, unit, sphere_box_corner) {
    Contact c;
    const Real d = 0.25f;
    const Vec3 center(1 + d, 1 + d, 1 + d);
    const Real expectedSep = std::sqrt(3 * d * d) - 0.5f;
    collide::sphereVsBox(Vec3(0), kNoRot, kUnitBox, center, Sphere{ 0.5f }, Real(0), c);
    TST_REQUIRE(approx(c.separation, expectedSep));
    TST_REQUIRE(approxVec(c.normal, Vec3(0.57735f, 0.57735f, 0.57735f), 1e-3f));
    TST_REQUIRE(approxVec(c.point, Vec3(1, 1, 1)));
}

// Center inside: push out along the least-penetrated axis (here +x, the nearest face).
TST_CASE(physics, unit, sphere_box_inside) {
    Contact c;
    const Vec3 center(0.8f, 0.1f, -0.1f);                 // nearest face is +x (pen 0.2)
    TST_REQUIRE(collide::sphereVsBox(Vec3(0), kNoRot, kUnitBox, center, Sphere{ 0.3f }, Real(0), c));
    TST_REQUIRE(approxVec(c.normal, Vec3(1, 0, 0)));      // least-penetration axis
    // separation = -(distance to +x face) - radius = -(1 - 0.8) - 0.3 = -0.5.
    TST_REQUIRE(approx(c.separation, -0.5f));
    TST_REQUIRE(c.touching);
}

// Rotated box: a symmetric cube rotated 90° about z is geometrically identical, so an axis
// approach must give the same face result — verifies the world<->local transform round-trips.
TST_CASE(physics, unit, sphere_box_rotated) {
    Contact c;
    const Quat rot = glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1));
    TST_REQUIRE(!collide::sphereVsBox(Vec3(0), rot, kUnitBox, Vec3(0, 2, 0), Sphere{ 0.5f }, Real(0), c));
    TST_REQUIRE(approx(c.separation, 0.5f));
    TST_REQUIRE(approxVec(c.normal, Vec3(0, 1, 0), 1e-4f));

    // A 45° rotation about z: approach along the box's local +x (world diagonal) hits a face.
    const Quat rot45 = glm::angleAxis(glm::radians(45.0f), Vec3(0, 0, 1));
    const Vec3 dir = rot45 * Vec3(1, 0, 0);              // world direction of the local +x face
    TST_REQUIRE(collide::sphereVsBox(Vec3(0), rot45, kUnitBox, dir * 1.4f, Sphere{ 0.5f }, Real(0), c));
    TST_REQUIRE(approx(c.separation, -0.1f, 1e-3f));
    TST_REQUIRE(approxVec(c.normal, dir, 1e-3f));
}
