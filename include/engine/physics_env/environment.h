//
//  environment.h
//  engine::physics_env
//
//  A headless, ECS-free RL environment (Milestone 2, Phase D1): a thin layer over one
//  `PhysicsWorld` + an `Articulation` (e.g. a humanoid on a ground plane). It exposes the
//  RL loop — `reset(seed)` / `setAction(...)` / `step()` — plus **raw-state accessors** (joint
//  q/qd, root pose + twist, per-body contact flags). It deliberately carries **no reward,
//  termination, or task**: the downstream trainer composes its observation from the raw state and
//  computes reward/termination itself. Reset is **in-place** (no destroy/recreate).
//
//  Actuation is selectable (`EnvConfig::actionMode`): TORQUE control (raw joint torques) or
//  PD-TARGET control (the action is a desired joint position — revolute angle / ball orientation as
//  a rotation vector — tracked by a PD servo with `kp`/`kd`). Either way revolute joints take 1
//  value and ball joints 3; the action vector concatenates them in joint order. Per-DOF magnitude
//  is clamped by the solver's actuator `maxTorque`.
//

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

#include "engine/core/math/transform.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics/world.h"

namespace engine::physics_env {

// The RL action interpretation now lives in engine/physics/config.h (physics::ActionMode) so it can
// be part of the centralized SimConfig; alias it here for physics_env callers.
using ActionMode = physics::ActionMode;

// The env config = the model (articulation) + the centralized tuning knobs (physics::SimConfig).
// All physics/solver/actuation knobs live in `sim`; `WorldDef` is derived from it via toWorldDef().
struct EnvConfig {
    physics::ArticulationDef articulation;   // e.g. physics::makeHumanoid()
    physics::SimConfig       sim{};          // gravity, substeps, damping, actuation, ground, solver…
};

class Environment {
public:
    explicit Environment(EnvConfig config);

    // Optional randomization hook, invoked at the end of reset(seed) — the ONLY task-specific seam
    // (engine ships none; the trainer supplies domain randomization / initial-state noise here).
    void setResetHook(std::function<void(uint64_t seed, Environment&)> hook) { resetHook_ = std::move(hook); }

    void reset(uint64_t seed = 0);
    void setAction(std::span<const float> action);   // size must be actDim()
    void step();

    // --- dimensions ---
    size_t actDim() const { return actDim_; }

    // --- raw state (the observation primitives; compose the obs vector downstream) ---
    std::span<const physics::JointState> jointStates() const { return world_->jointStates(); }
    engine::Transform    rootPose() const;
    physics::Vec3        rootLinearVelocity() const;
    physics::Vec3        rootAngularVelocity() const;
    std::span<const uint8_t> bodyContactFlags() const { return contactFlags_; }  // per articulation body

    // --- optional default flat observation (convenience for tests/examples, NOT the contract) ---
    //  layout: root pos(3) + root quat(4) + root linVel(3) + root angVel(3)
    //          + joint q[nJoints] + joint qd[nJoints] + body contact flags[nBodies]
    size_t defaultObsDim() const;
    void   packDefaultObs(std::span<float> out) const;

    // Escape hatches for advanced/downstream use.
    physics::PhysicsWorld&        world()        { return *world_; }
    const physics::Articulation&  articulation() const { return articulation_; }

private:
    void applyInitialState();
    void updateContactFlags();

    EnvConfig                              config_;
    std::unique_ptr<physics::PhysicsWorld> world_;
    physics::Articulation                  articulation_;

    struct ActuatedJoint {
        physics::JointHandle handle;
        physics::JointType   type;
        int                  dof;       // revolute 1, ball 3
        int                  offset;    // start index in the action vector
    };
    std::vector<ActuatedJoint> actuated_;
    size_t                     actDim_ = 0;
    std::vector<uint8_t>       contactFlags_;   // per articulation body (1 = in contact)
    std::function<void(uint64_t, Environment&)> resetHook_;
};

} // namespace engine::physics_env
