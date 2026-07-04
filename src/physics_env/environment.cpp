//
//  environment.cpp
//  engine::physics_env
//

#include "engine/physics_env/environment.h"

#include <algorithm>

#include "engine/physics/physics.h"

namespace engine::physics_env {

Environment::Environment(EnvConfig config) : config_(std::move(config)) {
    physics::WorldDef wd;
    wd.gravity = config_.gravity;
    wd.velocityIterations = config_.velocityIterations;
    wd.substeps = config_.substeps;
    wd.linearDamping = config_.linearDamping;
    wd.angularDamping = config_.angularDamping;
    world_ = physics::createPhysicsWorld(config_.backend, wd);

    if (config_.groundPlane) {
        physics::BodyDef g;
        g.type = physics::BodyType::Static;
        g.collider.type = physics::ColliderDesc::Type::Plane;
        g.collider.plane = physics::Plane{ physics::Vec3(0, 1, 0), physics::Real(0) };
        g.material.friction = config_.groundFriction;
        world_->createBody(g);
    }

    articulation_ = physics::buildArticulation(*world_, config_.articulation);

    // Actuation on every non-fixed joint; build the action-vector layout (revolute 1 DOF, ball 3
    // DOF), concatenated in joint order. Torque or PDTarget per config.actionMode.
    int offset = 0;
    for (size_t k = 0; k < articulation_.joints.size(); ++k) {
        const physics::JointType type = config_.articulation.joints[k].type;
        if (type == physics::JointType::Fixed) continue;
        const int dof = (type == physics::JointType::Ball) ? 3 : 1;
        physics::Actuator a;
        if (config_.actionMode == ActionMode::PDTarget) {
            a.mode = physics::ActuatorMode::PDTarget;
            a.kp = config_.kp; a.kd = config_.kd;
            a.target = physics::Real(0); a.ballTarget = physics::Quat(1, 0, 0, 0);
        } else {
            a.mode = physics::ActuatorMode::Torque;
        }
        a.maxTorque = config_.maxTorque;
        world_->setJointActuator(articulation_.joints[k], a);
        actuated_.push_back({ articulation_.joints[k], type, dof, offset });
        offset += dof;
    }
    actDim_ = static_cast<size_t>(offset);
    contactFlags_.assign(articulation_.bodies.size(), 0);

    reset(0);
}

void Environment::applyInitialState() {
    // Restore each articulation body to its authored pose with zero velocity (in-place).
    for (size_t i = 0; i < articulation_.bodies.size(); ++i) {
        const physics::BodyDef& d = config_.articulation.bodies[i];
        world_->setBodyState(articulation_.bodies[i], d.position, d.orientation,
                             physics::Vec3(0), physics::Vec3(0));
    }
}

void Environment::reset(uint64_t seed) {
    applyInitialState();
    world_->clearState();
    // Reset pending actuator commands to neutral (torque 0 / target = rest) so a reset episode
    // starts from rest.
    for (const ActuatedJoint& aj : actuated_) {
        if (config_.actionMode == ActionMode::PDTarget) {
            if (aj.type == physics::JointType::Ball) world_->setJointBallTarget(aj.handle, physics::Quat(1, 0, 0, 0));
            else                                     world_->setJointTarget(aj.handle, physics::Real(0));
        } else {
            if (aj.type == physics::JointType::Ball) world_->setJointBallTorque(aj.handle, physics::Vec3(0));
            else                                     world_->setJointTorque(aj.handle, physics::Real(0));
        }
    }
    if (resetHook_) resetHook_(seed, *this);
    world_->refreshState();      // make obs valid before the first step
    updateContactFlags();
}

void Environment::setAction(std::span<const float> action) {
    if (config_.actionMode == ActionMode::PDTarget) {
        for (const ActuatedJoint& aj : actuated_) {
            if (aj.type == physics::JointType::Ball) {
                // 3 action values = a target orientation expressed as a rotation vector.
                const physics::Vec3 rv(action[aj.offset], action[aj.offset + 1], action[aj.offset + 2]);
                const physics::Real len = glm::length(rv);
                const physics::Quat q = (len < physics::Real(1e-9))
                    ? physics::Quat(1, 0, 0, 0)
                    : physics::Quat(glm::angleAxis(len, rv / len));
                world_->setJointBallTarget(aj.handle, q);
            } else {
                world_->setJointTarget(aj.handle, action[aj.offset]);
            }
        }
        return;
    }
    for (const ActuatedJoint& aj : actuated_) {
        if (aj.type == physics::JointType::Ball) {
            world_->setJointBallTorque(aj.handle, physics::Vec3(action[aj.offset],
                                                                action[aj.offset + 1],
                                                                action[aj.offset + 2]));
        } else {
            world_->setJointTorque(aj.handle, action[aj.offset]);
        }
    }
}

void Environment::step() {
    world_->step(config_.controlDt);
    updateContactFlags();
}

void Environment::updateContactFlags() {
    std::fill(contactFlags_.begin(), contactFlags_.end(), static_cast<uint8_t>(0));
    for (const physics::ContactEvent& e : world_->contacts()) {
        for (size_t i = 0; i < articulation_.bodies.size(); ++i) {
            const uint32_t bi = articulation_.bodies[i].index;
            if (e.a.index == bi || e.b.index == bi) contactFlags_[i] = 1;
        }
    }
}

engine::Transform Environment::rootPose() const {
    return world_->pose(articulation_.bodies.front());
}
physics::Vec3 Environment::rootLinearVelocity() const {
    return world_->linearVelocities()[articulation_.bodies.front().index];
}
physics::Vec3 Environment::rootAngularVelocity() const {
    return world_->angularVelocities()[articulation_.bodies.front().index];
}

size_t Environment::defaultObsDim() const {
    size_t dofs = 0;   // generalized position DOFs (== velocity DOFs) across actuated joint types
    for (const physics::JointSpec& js : config_.articulation.joints) {
        if (js.type == physics::JointType::Revolute) dofs += 1;
        else if (js.type == physics::JointType::Ball) dofs += 3;
    }
    // root pose(7) + twist(6) + joint positions + joint velocities + per-body contact flags
    return 13 + 2 * dofs + articulation_.bodies.size();
}

void Environment::packDefaultObs(std::span<float> out) const {
    size_t o = 0;
    const engine::Transform root = rootPose();
    out[o++] = root.position.x; out[o++] = root.position.y; out[o++] = root.position.z;
    out[o++] = root.rotation.w; out[o++] = root.rotation.x; out[o++] = root.rotation.y; out[o++] = root.rotation.z;
    const physics::Vec3 lv = rootLinearVelocity();
    out[o++] = lv.x; out[o++] = lv.y; out[o++] = lv.z;
    const physics::Vec3 av = rootAngularVelocity();
    out[o++] = av.x; out[o++] = av.y; out[o++] = av.z;
    const auto js = jointStates();
    // Joint positions (revolute: angle; ball: rest-relative rotation vector), then velocities.
    for (size_t k = 0; k < js.size(); ++k) {
        const physics::JointType t = config_.articulation.joints[k].type;
        if (t == physics::JointType::Revolute) out[o++] = js[k].q;
        else if (t == physics::JointType::Ball) { out[o++] = js[k].rotation.x; out[o++] = js[k].rotation.y; out[o++] = js[k].rotation.z; }
    }
    for (size_t k = 0; k < js.size(); ++k) {
        const physics::JointType t = config_.articulation.joints[k].type;
        if (t == physics::JointType::Revolute) out[o++] = js[k].qd;
        else if (t == physics::JointType::Ball) { out[o++] = js[k].angularVelocity.x; out[o++] = js[k].angularVelocity.y; out[o++] = js[k].angularVelocity.z; }
    }
    for (uint8_t f : contactFlags_) out[o++] = static_cast<float>(f);
}

} // namespace engine::physics_env
