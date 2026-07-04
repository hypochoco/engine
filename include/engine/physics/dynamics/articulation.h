//
//  articulation.h
//  engine::physics / dynamics
//
//  Plain-data articulation description + builder (Milestone 2, Phase B4). An `ArticulationDef` is
//  a flat list of bodies plus joints that reference bodies BY INDEX (so a description is
//  self-contained and serializable, independent of any world). `buildArticulation` instantiates
//  it into a `PhysicsWorld`, returning the created handles. `makeHumanoid` is a ready preset.
//  Backend-agnostic: the same description builds into any `PhysicsWorld` backend.
//

#pragma once

#include <cstdint>
#include <vector>

#include "engine/physics/world.h"

namespace engine::physics {

// A joint referencing two bodies by their index in ArticulationDef::bodies (mirrors JointDef).
struct JointSpec {
    JointType type = JointType::Ball;
    uint32_t  bodyA = 0;
    uint32_t  bodyB = 0;
    Vec3      localAnchorA{0};
    Vec3      localAnchorB{0};
    Vec3      localAxisA{0, 0, 1};
    Vec3      localAxisB{0, 0, 1};
    bool      enableLimit = false;
    Real      lowerLimit = 0;
    Real      upperLimit = 0;
    Actuator  actuator{};
};

struct ArticulationDef {
    std::vector<BodyDef>   bodies;
    std::vector<JointSpec> joints;
};

// Handles of the instantiated articulation, index-aligned with the def's bodies/joints.
struct Articulation {
    std::vector<BodyHandle>  bodies;
    std::vector<JointHandle> joints;
};

// Create all bodies then all joints in `world`; joint specs' body indices map to created handles.
Articulation buildArticulation(PhysicsWorld& world, const ArticulationDef& def);

// ~13-limb humanoid preset (capsule/box limbs; ball waist/hips/shoulders, fixed neck, hinge
// elbows/knees/ankles with limits). All limbs share `limbCategory` and mask it out, so they do
// NOT self-collide (they'd fight the joints) but still collide with everything else (the ground).
// `rootPosition` places the pelvis.
ArticulationDef makeHumanoid(Vec3 rootPosition = Vec3(0, 1.0f, 0), uint32_t limbCategory = 0x0002);

} // namespace engine::physics
