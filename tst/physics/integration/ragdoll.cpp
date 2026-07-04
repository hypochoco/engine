//
//  ragdoll.cpp
//  engine::tst / physics / integration
//
//  Milestone 2, Phase B4: the humanoid preset built as a passive ragdoll (no actuation) collapses
//  onto the ground plane and comes to rest — no explosion (finite, bounded), settles (velocities
//  decay toward zero), and doesn't fall through the floor. Exercises joints + contacts + friction
//  + self-collision filtering together through the articulation builder.
//

#include <cmath>
#include <cstdio>

#include "engine/physics/dynamics/articulation.h"
#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

TST_CASE(physics, integration, ragdoll_settles) {
    WorldDef wd;
    wd.gravity = Vec3(0, -9.81f, 0);
    wd.velocityIterations = 16;
    wd.substeps = 4;
    wd.linearDamping = 0.2f;    // mild drag so free/undamped joint DOFs settle
    wd.angularDamping = 0.8f;
    auto w = createPhysicsWorld(Backend::Realtime, wd);

    // Ground plane (default category ⇒ collides with the humanoid limbs).
    {
        BodyDef g;
        g.type = BodyType::Static;
        g.collider.type = ColliderDesc::Type::Plane;
        g.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
        g.material.friction = 0.9f;
        w->createBody(g);
    }

    const Articulation body = buildArticulation(*w, makeHumanoid(Vec3(0, 1.05f, 0)));
    TST_REQUIRE(body.bodies.size() == 13);
    TST_REQUIRE(body.joints.size() == 12);
    for (const JointHandle j : body.joints) TST_REQUIRE(j.valid());

    for (int i = 0; i < 1200; ++i) w->step(1.0f / 120.0f);   // 10 s: collapse + settle

    const auto poses = w->poses();
    const auto lin   = w->linearVelocities();
    const auto ang   = w->angularVelocities();

    Real maxSpeed = 0, maxOmega = 0, minY = 1e30f, maxAbs = 0;
    for (const BodyHandle h : body.bodies) {
        const Vec3 p = poses[h.index].position;
        TST_REQUIRE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));   // no NaN blow-up
        maxAbs   = std::max(maxAbs, glm::length(p));
        minY     = std::min(minY, p.y);
        maxSpeed = std::max(maxSpeed, glm::length(lin[h.index]));
        maxOmega = std::max(maxOmega, glm::length(ang[h.index]));
    }
    std::printf("ragdoll: maxSpeed=%.4f maxOmega=%.4f minY=%.4f maxAbs=%.4f\n",
                maxSpeed, maxOmega, minY, maxAbs);

    TST_REQUIRE(maxAbs < 20.0f);      // stayed local (didn't fly off)
    TST_REQUIRE(minY   > -0.3f);      // didn't sink through the floor
    TST_REQUIRE(maxSpeed < 0.5f);     // settled (linear)
    TST_REQUIRE(maxOmega < 1.0f);     // settled (angular)
}
