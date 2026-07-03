//
//  physics_milestone.cpp
//  engine::tst
//
//  The driving milestone (headless, physics side): a sphere under gravity on an inclined
//  plane, stepped through the ECS bridge (RigidBody + physicsStep/Sync systems + Schedule).
//  Verifies it (a) descends, (b) travels down-slope, and (c) ROLLS (non-zero angular velocity,
//  spin axis perpendicular to the descent plane) rather than merely sliding.
//

#include <cassert>
#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/core/math/transform.h"
#include "engine/ecs/ecs.h"
#include "engine/physics/world.h"
#include "engine/physics_ecs/components.h"
#include "engine/physics_ecs/systems.h"

int main() {
    using namespace engine;
    using namespace engine::physics;

    WorldDef wd;
    wd.gravity = Vec3(0, -9.81f, 0);
    wd.velocityIterations = 12;
    wd.substeps = 2;
    auto world = createPhysicsWorld(Backend::Realtime, wd);

    // Inclined plane: normal tilted 30deg about z, surface through the origin.
    const float angle = glm::radians(30.0f);
    const Vec3 n = glm::normalize(Vec3(std::sin(angle), std::cos(angle), 0.0f));
    BodyDef plane;
    plane.type = BodyType::Static;
    plane.collider.type = ColliderDesc::Type::Plane;
    plane.collider.plane = Plane{ n, 0.0f };
    plane.material.friction = 0.9f;
    world->createBody(plane);

    // Sphere resting on the incline (center one radius up the normal from the surface).
    const float r = 0.5f;
    BodyDef ball;
    ball.type = BodyType::Dynamic;
    ball.mass = 1.0f;
    ball.collider.type = ColliderDesc::Type::Sphere;
    ball.collider.sphere = Sphere{ r };
    ball.material.friction = 0.9f;
    ball.material.restitution = 0.0f;
    ball.position = n * r;
    const BodyHandle ballHandle = world->createBody(ball);

    // --- drive it through the ECS bridge + scheduler ---
    ecs::World ecsWorld;
    ecs::Entity e = ecsWorld.spawn(engine::Transform{ .position = ball.position },
                                   physics_ecs::RigidBody{ ballHandle });
    ecsWorld.setResource(physics_ecs::PhysicsWorldRef{ world.get() });
    ecsWorld.setResource(physics_ecs::FixedStep{ 1.0f / 120.0f });

    ecs::Schedule sim;
    sim.add("physics.step", physics_ecs::stepSystem);
    sim.add("physics.sync", physics_ecs::syncSystem);

    const engine::Transform start = *ecsWorld.get<engine::Transform>(e);

    const Vec3 descent = glm::normalize(wd.gravity - glm::dot(wd.gravity, n) * n);
    for (int i = 0; i < 240; ++i) sim.run(ecsWorld);   // ~2 s

    const engine::Transform end = *ecsWorld.get<engine::Transform>(e);
    const Vec3 disp = end.position - start.position;
    const float along = glm::dot(disp, descent);
    const Vec3 omega = world->angularVelocities()[ballHandle.index];

    // The ECS Transform must track the physics world's pose (bridge sync).
    const engine::Transform worldPose = world->pose(ballHandle);
    assert(glm::length(end.position - worldPose.position) < 1e-4f);

    std::printf("descended dy = %.3f, down-slope = %.3f, |omega| = %.3f, omega.z = %.3f\n",
                disp.y, along, glm::length(omega), omega.z);

    assert(end.position.y < start.position.y - 0.2f);   // (a) descended
    assert(along > 0.5f);                               // (b) traveled down-slope
    assert(glm::length(omega) > 0.5f);                  // (c) it is rolling (spinning)
    // motion is in the x-y plane, so the roll axis is z.
    assert(std::fabs(omega.z) > 0.9f * glm::length(omega));

    std::printf("physics milestone ok (ball rolled down the incline)\n");
    return 0;
}
