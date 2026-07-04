//
//  articulation.cpp
//  engine::physics_ecs
//

#include "engine/physics_ecs/articulation.h"

#include "engine/core/math/transform.h"
#include "engine/ecs/ecs.h"
#include "engine/physics/world.h"
#include "engine/physics_ecs/components.h"

namespace engine::physics_ecs {

ArticulationEntities spawnArticulation(ecs::World& ecsWorld, physics::PhysicsWorld& pw,
                                       const physics::ArticulationDef& def) {
    ArticulationEntities out;
    out.articulation = physics::buildArticulation(pw, def);

    out.bodyEntities.reserve(out.articulation.bodies.size());
    for (const physics::BodyHandle h : out.articulation.bodies) {
        engine::Transform t = pw.pose(h);   // pose = position + orientation; scale stays 1
        out.bodyEntities.push_back(ecsWorld.spawn(RigidBody{ h }, t));
    }

    out.jointEntities.reserve(out.articulation.joints.size());
    for (const physics::JointHandle jh : out.articulation.joints)
        out.jointEntities.push_back(ecsWorld.spawn(Joint{ jh }, JointCommand{}));

    return out;
}

} // namespace engine::physics_ecs
