//
//  articulation_ecs.cpp
//  engine::tst / physics / integration
//
//  Milestone 2, Phase B4: the ECS ↔ physics articulation bridge. spawnArticulation builds the
//  humanoid in the PhysicsWorld and mirrors it into the ECS (an entity per body with RigidBody +
//  Transform, an entity per joint with Joint + JointCommand). A schedule of
//  actuatorFlush → step → sync advances physics and copies poses back into Transforms.
//

#include <cmath>
#include <cstdio>

#include "engine/core/math/transform.h"
#include "engine/ecs/ecs.h"
#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "engine/physics_ecs/articulation.h"
#include "engine/physics_ecs/components.h"
#include "engine/physics_ecs/systems.h"
#include "harness/harness.h"

using namespace engine;

TST_CASE(physics, integration, articulation_ecs) {
    physics::WorldDef wd;
    wd.gravity = physics::Vec3(0, -9.81f, 0);
    wd.velocityIterations = 16;
    wd.substeps = 4;
    wd.linearDamping = 0.2f;
    wd.angularDamping = 0.8f;
    auto pw = physics::createPhysicsWorld(physics::Backend::Realtime, wd);
    {
        physics::BodyDef g;
        g.type = physics::BodyType::Static;
        g.collider.type = physics::ColliderDesc::Type::Plane;
        g.collider.plane = physics::Plane{ physics::Vec3(0, 1, 0), 0.0f };
        g.material.friction = 0.9f;
        pw->createBody(g);
    }

    ecs::World world;
    world.setResource(physics_ecs::PhysicsWorldRef{ pw.get() });
    world.setResource(physics_ecs::FixedStep{ 1.0f / 120.0f });

    const physics_ecs::ArticulationEntities h =
        physics_ecs::spawnArticulation(world, *pw, physics::makeHumanoid(physics::Vec3(0, 1.05f, 0)));

    TST_REQUIRE(h.bodyEntities.size() == 13);
    TST_REQUIRE(h.jointEntities.size() == 12);
    // Body entities carry RigidBody + Transform; joint entities carry Joint + JointCommand.
    TST_REQUIRE(world.get<physics_ecs::RigidBody>(h.bodyEntities[0]) != nullptr);
    TST_REQUIRE(world.get<engine::Transform>(h.bodyEntities[0]) != nullptr);
    TST_REQUIRE(world.get<physics_ecs::Joint>(h.jointEntities[0]) != nullptr);
    TST_REQUIRE(world.get<physics_ecs::JointCommand>(h.jointEntities[0]) != nullptr);

    ecs::Schedule sched;
    sched.add("actuators", physics_ecs::actuatorFlushSystem);
    sched.add("step", physics_ecs::stepSystem);
    sched.add("sync", physics_ecs::syncSystem);
    for (int i = 0; i < 600; ++i) sched.run(world);   // 5 s

    // Every body entity's Transform matches its PhysicsWorld pose (sync worked) and is finite.
    physics::Real maxErr = 0, minY = 1e30f;
    for (size_t k = 0; k < h.bodyEntities.size(); ++k) {
        const engine::Transform* t = world.get<engine::Transform>(h.bodyEntities[k]);
        const physics_ecs::RigidBody* rb = world.get<physics_ecs::RigidBody>(h.bodyEntities[k]);
        TST_REQUIRE(t != nullptr && rb != nullptr);
        const engine::Transform pose = pw->pose(rb->body);
        maxErr = std::max(maxErr, glm::length(t->position - pose.position));
        minY = std::min(minY, t->position.y);
        TST_REQUIRE(std::isfinite(t->position.y));
    }
    std::printf("articulation_ecs: sync maxErr=%.6f minY=%.4f\n", maxErr, minY);
    TST_REQUIRE(maxErr < 1e-5f);   // Transforms track physics poses exactly
    TST_REQUIRE(minY > -0.3f);     // settled on the floor, didn't sink through
}
