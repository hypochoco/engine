//
//  manifolds.cpp
//  engine::tst / physics / unit
//
//  Contact-manifold generation against a half-space plane. The *count* and *depths* of contacts
//  drive stable resting: a flat box must yield 4 corners, an edge-balanced box 2, a deeply
//  buried box the 4 deepest (never more than 4); a lying capsule yields 2 endpoints, a standing
//  one just its lower cap.
//

#include <cmath>
#include <cstdio>

#include <glm/gtc/quaternion.hpp>

#include "engine/physics/collision/capsule.h"
#include "engine/physics/collision/primitives.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
const Plane kGround{ Vec3(0, 1, 0), Real(0) };
} // namespace

// Box resting flat: all 4 bottom corners touch at separation 0.
TST_CASE(physics, unit, box_plane_flat) {
    const Box box{ Vec3(0.5f) };
    Contact cs[4];
    const int n = collide::boxVsPlane(Vec3(0, 0.5f, 0), Quat(1, 0, 0, 0), box, kGround, Real(0), cs);
    TST_REQUIRE(n == 4);
    for (int i = 0; i < n; ++i) {
        TST_REQUIRE(std::fabs(cs[i].separation) < 1e-5f);
        TST_REQUIRE(std::fabs(cs[i].point.y) < 1e-5f);    // corners sit on the plane
    }
}

// Box balanced on an edge (45° about z): exactly the 2 lowest corners (spanning ±z) touch.
TST_CASE(physics, unit, box_plane_edge) {
    const Box box{ Vec3(0.5f) };
    const Quat rot = glm::angleAxis(glm::radians(45.0f), Vec3(0, 0, 1));
    const Real lowest = 0.5f * std::sqrt(2.0f);           // lowest corner offset below center
    Contact cs[4];
    const int n = collide::boxVsPlane(Vec3(0, lowest, 0), rot, box, kGround, Real(0), cs);
    TST_REQUIRE(n == 2);
    for (int i = 0; i < n; ++i) TST_REQUIRE(std::fabs(cs[i].separation) < 1e-4f);
}

// Box buried well below the plane: 8 corners penetrate but only the 4 deepest are kept.
TST_CASE(physics, unit, box_plane_deep) {
    const Box box{ Vec3(0.5f) };
    Contact cs[4];
    const int n = collide::boxVsPlane(Vec3(0, -1.0f, 0), Quat(1, 0, 0, 0), box, kGround, Real(0), cs);
    TST_REQUIRE(n == 4);                                  // capped at 4, not 8
    // Deepest corners = bottom face at y = -1.5, so separation = -1.5.
    for (int i = 0; i < n; ++i) TST_REQUIRE(std::fabs(cs[i].separation + 1.5f) < 1e-4f);
}

// A lying capsule (axis along X) contacts the plane with BOTH endpoints -> 2-point manifold.
TST_CASE(physics, unit, capsule_plane_lying) {
    const Capsule cap{ 0.4f, 0.6f };
    const Quat lying = glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1));   // local +Y -> world X
    Contact cs[2];
    const int n = collide::capsuleVsPlane(Vec3(0, 0.4f, 0), lying, cap, kGround, Real(0), cs);
    TST_REQUIRE(n == 2);
    for (int i = 0; i < n; ++i) {
        TST_REQUIRE(std::fabs(cs[i].separation) < 1e-4f);
        TST_REQUIRE(std::fabs(cs[i].point.y) < 1e-4f);
    }
}

// A standing capsule (axis along Y): only the lower cap is within reach -> 1 contact.
TST_CASE(physics, unit, capsule_plane_standing) {
    const Capsule cap{ 0.4f, 0.6f };
    Contact cs[2];
    // Lower endpoint at y = 1.0 - 0.6 = 0.4; its surface reaches y = 0 exactly.
    const int n = collide::capsuleVsPlane(Vec3(0, 1.0f, 0), Quat(1, 0, 0, 0), cap, kGround, Real(0), cs);
    TST_REQUIRE(n == 1);
    TST_REQUIRE(std::fabs(cs[0].separation) < 1e-4f);
}
