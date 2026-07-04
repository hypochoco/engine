//
//  world.h
//  engine::physics
//
//  The runtime-virtual PhysicsWorld interface (physics plan §1, §4): a coarse boundary
//  (createBody / step / bulk readback) so multiple backends can coexist in one process, while
//  each backend's hot loops stay concrete and data-oriented. Phase-1 colliders are sphere +
//  plane; more shapes arrive with GJK/EPA in Phase 2.
//

#pragma once

#include <memory>
#include <span>

#include "engine/core/math/transform.h"
#include "engine/core/memory/handle.h"
#include "engine/physics/dynamics/body.h"
#include "engine/physics/shapes/shapes.h"
#include "engine/physics/types.h"

namespace engine::core { class ThreadPool; }

namespace engine::physics {

using BodyHandle = core::Handle<struct BodyTag>;

struct ColliderDesc {
    enum class Type { Sphere, Plane, Box, ConvexHull, Capsule } type = Type::Sphere;
    Sphere     sphere{};
    Plane      plane{};
    Box        box{};
    ConvexHull convexHull{};
    Capsule    capsule{};
};

struct BodyDef {
    Vec3            position{0};
    Quat            orientation{1, 0, 0, 0};
    Vec3            linearVelocity{0};
    Vec3            angularVelocity{0};
    Real            mass = Real(1);         // ignored for Static/Kinematic
    ColliderDesc    collider{};
    PhysicsMaterial material{};
    BodyType        type = BodyType::Dynamic;
    // Collision filtering: bodies A and B collide only if (A.category & B.mask) and
    // (B.category & A.mask) are both non-zero. Default = collide with everything. Used to stop an
    // articulation's jointed limbs from colliding with each other (they'd fight the joints).
    uint32_t        collisionCategory = 0x0001;
    uint32_t        collisionMask     = 0xFFFFFFFFu;
};

enum class BroadphaseKind { SweepAndPrune, UniformGrid };

struct WorldDef {
    Vec3               gravity{0, Real(-9.81), 0};
    int                velocityIterations = 8;
    int                substeps = 1;
    BroadphaseKind     broadphase = BroadphaseKind::UniformGrid;

    // Optional velocity damping (models drag; per second). 0 = none (default). Applied to dynamic
    // bodies each substep so free/undamped DOFs (e.g. a ragdoll's spinning limb) still settle.
    Real               linearDamping = Real(0);
    Real               angularDamping = Real(0);

    // Optional: if set, the step parallelizes its embarrassingly-parallel stages (integration,
    // narrowphase) across the pool when body/pair counts exceed `parallelThreshold`. Results
    // are identical to serial (deterministic). The broadphase sort and the contact solver
    // remain serial for now.
    core::ThreadPool*  threadPool = nullptr;
    int                parallelThreshold = 4096;

    // Continuous collision detection: swept broadphase AABBs + speculative contacts so fast
    // bodies don't tunnel through finite colliders in one step.
    bool               continuousDetection = true;
};

struct ContactEvent {
    BodyHandle a, b;
    Vec3       point{0};
    Vec3       normal{0, 1, 0};
    Real       separation = 0;
};

// --- Articulation (Milestone 2, Phase B): maximal-coordinate joint constraints ---------------
// A joint is a persistent bilateral constraint between two bodies, solved in the same
// sequential-impulse loop as contacts (see the articulation decision note). The API is kept
// backend-agnostic so a future reduced-coordinate (Featherstone) backend implements the same
// surface. Frames are given per body in that body's LOCAL space (relative to its center of
// mass / origin); the constraint drives the two world-space frames together.
using JointHandle = core::Handle<struct JointTag>;

enum class JointType {
    Ball,       // spherical / point-to-point: 3 DOF removed (anchors coincide)
    Revolute,   // hinge: point-to-point + rotation locked to a shared axis (5 removed)
    Fixed,      // weld: point-to-point + relative orientation locked (6 removed)
};

// Actuator (Phase B3) — the RL action write-path for a joint DOF. For a Revolute joint this acts
// about the hinge axis (1-DOF). `maxTorque <= 0` means unlimited.
enum class ActuatorMode { None, Torque, PDTarget };

struct Actuator {
    ActuatorMode mode = ActuatorMode::None;
    Real kp = 0;          // PD position gain
    Real kd = 0;          // PD velocity/damping gain
    Real target = 0;      // Revolute PDTarget: desired joint angle q (radians)
    Real targetVel = 0;   // Revolute PDTarget: desired joint rate qd
    Real torque = 0;      // Revolute Torque mode: commanded joint torque
    Real maxTorque = 0;   // clamp on |applied torque|; <= 0 ⇒ unlimited
    // Ball joints (3-DOF spherical actuation):
    Quat ballTarget{1, 0, 0, 0};   // PDTarget: desired relative orientation (B in A's frame)
    Vec3 ballTorque{0};            // Torque mode: commanded 3-DOF torque (world frame)
};

// Joint state (Phase B3) — the observation primitive. For a Revolute joint `q` is the hinge angle
// (radians, from the creation-time reference) and `qd` the angular rate about the axis. For a Ball
// joint the multi-DOF fields carry the state: `rotation` is the rest-relative orientation as a
// rotation vector (axis·angle) and `angularVelocity` the relative angular velocity; `q`/`qd` are 0.
// Fixed joints leave everything 0.
struct JointState {
    Real q  = 0;
    Real qd = 0;
    Vec3 rotation{0};          // Ball: rest-relative orientation as a rotation vector
    Vec3 angularVelocity{0};   // Ball: relative angular velocity (child frame)
};

struct JointDef {
    JointType  type = JointType::Ball;
    BodyHandle a;
    BodyHandle b;
    // Anchor point in each body's local frame. For a well-posed joint the two anchors should
    // coincide in world space at creation (the solver removes any initial error via Baumgarte).
    Vec3       localAnchorA{0};
    Vec3       localAnchorB{0};
    // Hinge axis in each body's local frame (Revolute only; unit-length). The two axes should
    // be world-aligned at creation; the solver keeps them aligned.
    Vec3       localAxisA{0, 0, 1};
    Vec3       localAxisB{0, 0, 1};
    // Angular limits about the hinge axis (Revolute only, radians, relative to the creation-time
    // reference). Enforced as a one-sided constraint when `enableLimit` and lower <= upper.
    bool       enableLimit = false;
    Real       lowerLimit = 0;
    Real       upperLimit = 0;
    // Optional actuator (Revolute; acts about the hinge axis).
    Actuator   actuator{};
};

class PhysicsWorld {
public:
    virtual ~PhysicsWorld() = default;

    virtual BodyHandle createBody(const BodyDef&) = 0;
    virtual void       destroyBody(BodyHandle)    = 0;
    virtual void       setGravity(Vec3)           = 0;
    virtual void       step(Real dt)              = 0;

    // Joints (Phase B). createJoint returns an invalid handle if either body handle is stale.
    virtual JointHandle createJoint(const JointDef&) = 0;
    virtual void        destroyJoint(JointHandle)    = 0;

    // Actuators (Phase B3) — the action write-path. Per-joint convenience setters plus batched
    // SoA writes indexed by JointHandle.index (for vectorized envs). setJointActuator replaces
    // the whole actuator config; setJointTarget/Torque update just the command each step.
    virtual void setJointActuator(JointHandle, const Actuator&) = 0;
    virtual void setJointTarget(JointHandle, Real target)       = 0;   // PDTarget command (revolute)
    virtual void setJointTorque(JointHandle, Real torque)       = 0;   // Torque command (revolute)
    virtual void setJointBallTorque(JointHandle, Vec3 torque)   = 0;   // Torque command (ball, 3-DOF)
    virtual void setJointBallTarget(JointHandle, Quat target)   = 0;   // PDTarget command (ball, orientation)
    virtual void setJointTargets(std::span<const Real> targets) = 0;   // bulk PDTarget commands
    virtual void setJointTorques(std::span<const Real> torques) = 0;   // bulk Torque commands

    // Joint state read-path (Phase B3) — the observation. Single + bulk (indexed by handle.index).
    virtual JointState jointState(JointHandle) const           = 0;
    virtual std::span<const JointState> jointStates() const    = 0;

    // In-place reset support (Phase D env). setBodyState overwrites a body's pose + velocity;
    // clearState zeros solver warm-start (joint accumulators + cached joint states) and contact
    // events — together they restart an episode deterministically without destroy/recreate.
    virtual void setBodyState(BodyHandle, const Vec3& position, const Quat& orientation,
                              const Vec3& linearVelocity, const Vec3& angularVelocity) = 0;
    virtual void clearState() = 0;
    // Recompute bulk readback (poses/velocities/joint q,qd) from current state WITHOUT advancing
    // time — so observations are valid immediately after an in-place reset (before the first step).
    virtual void refreshState() = 0;

    // Bulk, index-stable readback (indexed by BodyHandle.index) — one virtual call per array,
    // no per-body dispatch even at scale (§4).
    virtual std::span<const engine::Transform> poses() const              = 0;
    virtual std::span<const Vec3>               linearVelocities() const  = 0;
    virtual std::span<const Vec3>               angularVelocities() const = 0;

    // Single-body convenience.
    virtual engine::Transform pose(BodyHandle) const = 0;

    virtual std::span<const ContactEvent> contacts() const = 0;
};

enum class Backend {
    Realtime,   // maximal-coordinate sequential-impulse solver (contacts + joint constraints)
    Reduced,    // reduced-coordinate Featherstone/ABA articulation (Phase E)
};

std::unique_ptr<PhysicsWorld> createPhysicsWorld(Backend backend, const WorldDef& def);

} // namespace engine::physics
