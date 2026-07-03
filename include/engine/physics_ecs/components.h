//
//  components.h
//  engine::physics_ecs
//
//  ECS components + resources that bridge the ECS to a PhysicsWorld. Per the Q2 decision the
//  ECS does NOT own the pose: `RigidBody` is just a handle into the world's arrays, and
//  `Transform` (from core) stays the separate, universal spatial component. Trivially
//  copyable (ECS rule).
//

#pragma once

#include "engine/physics/world.h"

namespace engine::physics_ecs {

// Links an entity to its body in the PhysicsWorld. No pose stored here (backend owns state).
struct RigidBody {
    physics::BodyHandle body;
};

// Resource: a non-owning pointer to the world the systems drive (the app owns the world).
struct PhysicsWorldRef {
    physics::PhysicsWorld* world = nullptr;
};

// Resource: the fixed simulation timestep.
struct FixedStep {
    physics::Real dt = physics::Real(1) / physics::Real(120);
};

} // namespace engine::physics_ecs
