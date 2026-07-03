//
//  systems.h
//  engine::physics_ecs
//
//  Schedule-compatible systems (void(ecs::World&)) that bridge the ECS and the PhysicsWorld.
//  Add them in order to an ecs::Schedule; both read a PhysicsWorldRef resource.
//

#pragma once

#include "engine/ecs/world.h"

namespace engine::physics_ecs {

// Advances the physics world by one fixed step (reads PhysicsWorldRef + FixedStep resources).
void stepSystem(ecs::World& world);

// Copies world body poses into each entity's Transform (reads PhysicsWorldRef).
void syncSystem(ecs::World& world);

} // namespace engine::physics_ecs
