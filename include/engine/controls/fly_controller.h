//
//  fly_controller.h
//  engine::controls
//
//  Input-driven ECS controllers. `FlyController` is a component; `flyControllerSystem` is a
//  `void(World&)` system that reads the input::InputState + engine::Time resources and drives
//  the entity's engine::Transform (WASD move, Q/E or Space/LeftCtrl down/up, LeftShift sprint,
//  right-mouse look). The controller only touches Transform — it neither knows nor cares that
//  the entity is a camera; a <Transform, Camera, FlyController> entity is a flyable camera.
//
//  Lives in engine::controls (deps: ecs + input + core, NO graphics) so input→Transform logic
//  stays out of both the foundational input module and the graphics-coupled scene module.
//

#pragma once

namespace engine::ecs { class World; }

namespace engine::controls {

struct FlyController {
    float yaw              = -90.0f;   // degrees; -90 looks down -Z
    float pitch            = -12.0f;   // degrees; + looks up
    float moveSpeed        = 6.0f;     // units / second
    float sprintMultiplier = 3.0f;
    float lookSensitivity  = 0.12f;    // degrees / pixel of mouse motion
};

// Drives every <Transform, FlyController> entity from the input::InputState + engine::Time
// resources. No-op if either resource is absent.
void flyControllerSystem(ecs::World& world);

} // namespace engine::controls
