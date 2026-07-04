//
//  collision_filter.cpp
//  engine::tst / physics / unit
//
//  Category/mask collision filtering (Milestone 2, Phase B4): two overlapping dynamic spheres
//  collide by default, but if they share a category that each masks out, the contact is skipped
//  (used so an articulation's jointed limbs don't fight their joints). Verified by whether an
//  overlapping pair pushes apart (unfiltered) or passes through / stays put (filtered).
//

#include <cstdio>

#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {

// Two overlapping spheres, no gravity; step and return their center separation along X after
// settling. `filter` sets both into category 0b10 with a mask that excludes 0b10.
Real separationAfter(bool filter) {
    WorldDef wd;
    wd.gravity = Vec3(0);
    wd.velocityIterations = 16;
    auto w = createPhysicsWorld(Backend::Realtime, wd);

    auto make = [&](Vec3 pos) {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.mass = 1.0f;
        d.position = pos;
        d.collider.type = ColliderDesc::Type::Sphere;
        d.collider.sphere = Sphere{ 0.5f };
        if (filter) { d.collisionCategory = 0b10; d.collisionMask = ~0b10u; }
        return w->createBody(d);
    };
    const BodyHandle a = make(Vec3(-0.2f, 0, 0));   // overlapping (centers 0.4 < sum radii 1.0)
    const BodyHandle b = make(Vec3(0.2f, 0, 0));

    for (int i = 0; i < 120; ++i) w->step(1.0f / 120.0f);
    return w->pose(b).position.x - w->pose(a).position.x;
}

} // namespace

TST_CASE(physics, unit, collision_filter) {
    const Real unfiltered = separationAfter(false);
    const Real filtered   = separationAfter(true);
    std::printf("filter: unfiltered sep = %.3f, filtered sep = %.3f\n", unfiltered, filtered);
    // Unfiltered: the overlap resolves → they push apart to ~touching (≈ sum of radii, 1.0).
    TST_REQUIRE(unfiltered > 0.9f);
    // Filtered: no contact is generated → they stay at their initial 0.4 separation.
    TST_REQUIRE(std::fabs(filtered - 0.4f) < 1e-3f);
}
