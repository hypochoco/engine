//
//  articulation.h
//  engine::physics_ecs
//
//  Bridges a plain-data physics::ArticulationDef into the ECS (Milestone 2, Phase B4): builds the
//  bodies + joints in the PhysicsWorld, then spawns one ECS entity per body (RigidBody + Transform,
//  so syncSystem drives it and the renderer can draw it) and one per joint (Joint + JointCommand,
//  the actuator write-path). Returns the created handles + entities.
//

#pragma once

#include <vector>

#include "engine/ecs/entity.h"
#include "engine/physics/dynamics/articulation.h"

namespace engine::ecs { class World; }
namespace engine::physics { class PhysicsWorld; }

namespace engine::physics_ecs {

struct ArticulationEntities {
    physics::Articulation    articulation;   // body/joint handles in the PhysicsWorld
    std::vector<ecs::Entity> bodyEntities;   // index-aligned with articulation.bodies
    std::vector<ecs::Entity> jointEntities;  // index-aligned with articulation.joints
};

ArticulationEntities spawnArticulation(ecs::World& ecsWorld, physics::PhysicsWorld& physicsWorld,
                                       const physics::ArticulationDef& def);

} // namespace engine::physics_ecs
