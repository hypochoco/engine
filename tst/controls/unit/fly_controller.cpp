#include "harness/harness.h"
//
//  fly_controller.cpp
//  engine::tst — controls / unit
//
//  Headless test of the ECS fly-controller structure: an InputState + Time resource drive a
//  <Transform, FlyController> entity via flyControllerSystem. No GLFW, no window.
//

#include <glm/glm.hpp>

#include "engine/core/core.h"          // engine::Transform, engine::Time
#include "engine/ecs/ecs.h"
#include "engine/input/input.h"
#include "engine/controls/fly_controller.h"

using namespace engine;

TST_CASE(controls, unit, fly_moves_transform) {
    ecs::World world;

    // Input resource: hold W this frame.
    input::InputState in;
    in.newFrame();
    in.setKey(input::Key::W, true);
    world.setResource(in);
    world.setResource(Time{.dt = 0.1f});

    // Camera-less flyable entity: Transform + FlyController (yaw -90, pitch 0 → forward -Z).
    ecs::Entity e = world.spawn(Transform{},
                                controls::FlyController{.yaw = -90.0f, .pitch = 0.0f, .moveSpeed = 10.0f});

    controls::flyControllerSystem(world);

    Transform* t = world.get<Transform>(e);
    TST_REQUIRE(t != nullptr);
    // W for 0.1s at speed 10 → 1 unit along forward (−Z).
    TST_APPROX(t->position.z, -1.0f, 1e-3);
    TST_APPROX(t->position.x, 0.0f, 1e-3);
    TST_APPROX(t->position.y, 0.0f, 1e-3);
}

TST_CASE(controls, unit, fly_look_updates_yaw) {
    ecs::World world;

    // Right button held + horizontal mouse delta → yaw turns; also verify the resulting
    // orientation looks roughly along the new forward.
    input::InputState in;
    in.newFrame();
    in.setMouseButton(input::MouseButton::Right, true);
    in.addMouseDelta({10.0f, 0.0f});
    world.setResource(in);
    world.setResource(Time{.dt = 0.016f});

    ecs::Entity e = world.spawn(Transform{}, controls::FlyController{.yaw = -90.0f, .pitch = 0.0f});
    controls::flyControllerSystem(world);

    // yaw advanced by delta.x * sensitivity (default 0.12).
    // Confirm the entity's rotation points forward near the expected direction: with a small
    // yaw change from -90°, forward stays close to -Z but gains a small +X (or -X) component.
    Transform* t = world.get<Transform>(e);
    TST_REQUIRE(t != nullptr);
    const glm::vec3 fwd = t->rotation * glm::vec3(0.0f, 0.0f, -1.0f);
    TST_REQUIRE(fwd.z < -0.9f);                 // still mostly forward
    TST_REQUIRE(std::fabs(fwd.x) > 1e-3);       // but turned off pure -Z
}
