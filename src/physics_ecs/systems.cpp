//
//  systems.cpp
//  engine::physics_ecs
//

#include "engine/physics_ecs/systems.h"

#include "engine/core/math/transform.h"
#include "engine/ecs/ecs.h"
#include "engine/physics_ecs/components.h"

namespace engine::physics_ecs {

void stepSystem(ecs::World& world) {
    auto* ref = world.getResource<PhysicsWorldRef>();
    auto* step = world.getResource<FixedStep>();
    if (!ref || !ref->world || !step) return;
    ref->world->step(step->dt);
}

void syncSystem(ecs::World& world) {
    auto* ref = world.getResource<PhysicsWorldRef>();
    if (!ref || !ref->world) return;
    physics::PhysicsWorld& pw = *ref->world;

    world.query<RigidBody, engine::Transform>().each(
        [&](ecs::Entity, RigidBody& rb, engine::Transform& t) {
            const engine::Transform p = pw.pose(rb.body);
            t.position = p.position;
            t.rotation = p.rotation;   // keep the entity's own scale
        });
}

} // namespace engine::physics_ecs
