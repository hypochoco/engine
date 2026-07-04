//
//  time.h
//  engine::core
//
//  A frame-time resource for variable-rate systems (e.g. camera controllers, animation).
//  Physics uses its own fixed timestep (physics_ecs::FixedStep); this is the wall-clock delta
//  of the current frame. Stored as an ECS World resource (setResource<engine::Time>).
//

#pragma once

namespace engine {

struct Time {
    float  dt      = 0.0f;   // seconds elapsed since the previous frame
    double elapsed = 0.0;    // seconds since start (optional; app-maintained)
};

} // namespace engine
