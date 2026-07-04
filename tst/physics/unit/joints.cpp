//
//  joints.cpp
//  engine::tst / physics / unit
//
//  Maximal-coordinate joint constraints (Milestone 2, Phase B1) on the sequential-impulse solver:
//    * Ball    — the two anchor points stay coincident (no drift under gravity).
//    * Revolute — relative rotation is confined to the hinge axis (off-axis rotation ≈ 0) while
//                 on-axis rotation is preserved.
//    * Fixed   — the child stays welded: a horizontal cantilever holds its pose against gravity
//                 (a ball joint would swing down; the fixed joint must not).
//  Each joint links a Static anchor body to a Dynamic body; colliders are tiny + far apart so no
//  contacts interfere (self-collision filtering is Phase B4).
//

#include <cmath>
#include <cstdio>
#include <span>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {

std::unique_ptr<PhysicsWorld> makeWorld(Vec3 gravity, int velIters, int substeps) {
    WorldDef wd;
    wd.gravity = gravity;
    wd.velocityIterations = velIters;
    wd.substeps = substeps;
    return createPhysicsWorld(Backend::Realtime, wd);
}

BodyHandle addBody(PhysicsWorld& w, BodyType type, Vec3 pos, Real mass,
                   Vec3 angVel = Vec3(0), Vec3 linVel = Vec3(0), Real radius = 0.1f) {
    BodyDef d;
    d.type = type;
    d.position = pos;
    d.mass = mass;
    d.angularVelocity = angVel;
    d.linearVelocity = linVel;
    d.collider.type = ColliderDesc::Type::Sphere;
    d.collider.sphere = Sphere{ radius };
    return w.createBody(d);
}

} // namespace

// Ball joint: a pendulum swinging under gravity keeps its pivot anchor at the world origin.
TST_CASE(physics, unit, ball_anchors_coincide) {
    auto w = makeWorld(Vec3(0, -9.81f, 0), 16, 4);
    const BodyHandle a = addBody(*w, BodyType::Static, Vec3(0), 0.0f);
    const BodyHandle b = addBody(*w, BodyType::Dynamic, Vec3(1, 0, 0), 1.0f);

    JointDef jd;
    jd.type = JointType::Ball;
    jd.a = a; jd.b = b;
    jd.localAnchorA = Vec3(0, 0, 0);    // pivot at A's origin (world origin)
    jd.localAnchorB = Vec3(-1, 0, 0);   // B's anchor is 1 unit toward the pivot
    const JointHandle j = w->createJoint(jd);
    TST_REQUIRE(j.valid());

    Real maxErr = 0;
    for (int i = 0; i < 360; ++i) {   // 3 s at 1/120
        w->step(1.0f / 120.0f);
        const engine::Transform tb = w->pose(b);
        const Vec3 worldAnchorB = tb.position + tb.rotation * jd.localAnchorB;
        maxErr = std::max(maxErr, glm::length(worldAnchorB));   // pivot should stay at origin
    }
    std::printf("ball: max anchor drift = %.5f\n", maxErr);
    TST_REQUIRE(maxErr < 0.02f);

    // Sanity: the pendulum actually moved (swung down under gravity), i.e. the joint didn't freeze it.
    const engine::Transform tb = w->pose(b);
    TST_REQUIRE(tb.position.y < -0.1f);
}

// Revolute joint about world +Z: an off-axis (X) initial spin is removed; on-axis (Z) spin persists.
TST_CASE(physics, unit, hinge_confines_to_axis) {
    auto w = makeWorld(Vec3(0), 24, 2);   // no gravity: isolate the angular constraint
    const BodyHandle a = addBody(*w, BodyType::Static, Vec3(0), 0.0f);
    // Consistent orbit about Z through the origin (ω_z=4 ⇒ COM vel = ω×r = (0,4,0)), plus an
    // off-axis X spin (3) the hinge must remove.
    const BodyHandle b = addBody(*w, BodyType::Dynamic, Vec3(1, 0, 0), 1.0f,
                                 Vec3(3, 0, 4), Vec3(0, 4, 0));

    JointDef jd;
    jd.type = JointType::Revolute;
    jd.a = a; jd.b = b;
    jd.localAnchorA = Vec3(0, 0, 0);
    jd.localAnchorB = Vec3(-1, 0, 0);
    jd.localAxisA = Vec3(0, 0, 1);
    jd.localAxisB = Vec3(0, 0, 1);
    const JointHandle j = w->createJoint(jd);
    TST_REQUIRE(j.valid());

    for (int i = 0; i < 240; ++i) w->step(1.0f / 120.0f);   // 2 s

    const engine::Transform tb = w->pose(b);
    // Rotation confined to Z ⇒ the quaternion's x,y components stay ≈ 0.
    std::printf("hinge: qB = (%.4f, %.4f, %.4f, %.4f)\n", tb.rotation.w, tb.rotation.x, tb.rotation.y, tb.rotation.z);
    TST_REQUIRE(std::fabs(tb.rotation.x) < 0.03f);
    TST_REQUIRE(std::fabs(tb.rotation.y) < 0.03f);
    // On-axis spin preserved ⇒ it rotated substantially about Z.
    TST_REQUIRE(std::fabs(tb.rotation.z) > 0.5f);
}

// Fixed joint: a horizontal cantilever holds its pose against gravity (a ball joint would fall).
TST_CASE(physics, unit, fixed_keeps_relative_pose) {
    auto w = makeWorld(Vec3(0, -9.81f, 0), 32, 4);
    const BodyHandle a = addBody(*w, BodyType::Static, Vec3(0), 0.0f);
    // r=0.5 body (invInertia=10, realistic vs a hair-trigger point mass) pinned 1 m from its COM.
    const BodyHandle b = addBody(*w, BodyType::Dynamic, Vec3(1, 0, 0), 1.0f,
                                 Vec3(0), Vec3(0), 0.5f);

    JointDef jd;
    jd.type = JointType::Fixed;
    jd.a = a; jd.b = b;
    jd.localAnchorA = Vec3(0, 0, 0);    // pivot at origin; B's COM is 1 unit out → gravity torque
    jd.localAnchorB = Vec3(-1, 0, 0);
    const JointHandle j = w->createJoint(jd);
    TST_REQUIRE(j.valid());

    for (int i = 0; i < 240; ++i) w->step(1.0f / 120.0f);   // 2 s

    const engine::Transform tb = w->pose(b);
    std::printf("fixed: pos = (%.4f, %.4f, %.4f), qxyz = (%.4f, %.4f, %.4f)\n",
                tb.position.x, tb.position.y, tb.position.z, tb.rotation.x, tb.rotation.y, tb.rotation.z);
    // Position held near (1,0,0) and orientation near identity (no swing-down).
    TST_REQUIRE(glm::length(tb.position - Vec3(1, 0, 0)) < 0.05f);
    TST_REQUIRE(std::fabs(tb.rotation.x) < 0.05f);
    TST_REQUIRE(std::fabs(tb.rotation.y) < 0.05f);
    TST_REQUIRE(std::fabs(tb.rotation.z) < 0.05f);
}

// --- B2: hinge angle limit ------------------------------------------------------------------

// A pendulum on a limited hinge swings down under gravity and STOPS at the lower clamp instead of
// hanging straight down.
TST_CASE(physics, unit, hinge_limit_stops) {
    auto w = makeWorld(Vec3(0, -9.81f, 0), 32, 4);
    const BodyHandle a = addBody(*w, BodyType::Static, Vec3(0), 0.0f);
    const BodyHandle b = addBody(*w, BodyType::Dynamic, Vec3(1, 0, 0), 1.0f, Vec3(0), Vec3(0), 0.5f);

    JointDef jd;
    jd.type = JointType::Revolute;
    jd.a = a; jd.b = b;
    jd.localAnchorA = Vec3(0, 0, 0);
    jd.localAnchorB = Vec3(-1, 0, 0);
    jd.localAxisA = Vec3(0, 0, 1);
    jd.localAxisB = Vec3(0, 0, 1);
    jd.enableLimit = true;
    jd.lowerLimit = -0.5f;   // swinging down drives q negative; clamp at -0.5 rad
    jd.upperLimit =  1.0f;
    const JointHandle j = w->createJoint(jd);
    TST_REQUIRE(j.valid());

    for (int i = 0; i < 600; ++i) w->step(1.0f / 120.0f);   // 5 s: swing + settle at the clamp

    const Real q = w->jointState(j).q;
    std::printf("hinge_limit: settled q = %.4f (clamp -0.5, free hang would be ~-1.57)\n", q);
    TST_REQUIRE(q > -0.60f);   // did not blow past the clamp
    TST_REQUIRE(q < -0.40f);   // rests against the clamp (not near the -π/2 free-hang)
}

// --- B3: actuators + q/qd read-path ---------------------------------------------------------

// PD "stand": a PD actuator holds a pendulum near a horizontal target angle against gravity.
TST_CASE(physics, unit, actuator_pd_holds_angle) {
    auto w = makeWorld(Vec3(0, -9.81f, 0), 32, 4);
    const BodyHandle a = addBody(*w, BodyType::Static, Vec3(0), 0.0f);
    const BodyHandle b = addBody(*w, BodyType::Dynamic, Vec3(1, 0, 0), 1.0f, Vec3(0), Vec3(0), 0.5f);

    JointDef jd;
    jd.type = JointType::Revolute;
    jd.a = a; jd.b = b;
    jd.localAnchorA = Vec3(0, 0, 0);
    jd.localAnchorB = Vec3(-1, 0, 0);
    jd.localAxisA = Vec3(0, 0, 1);
    jd.localAxisB = Vec3(0, 0, 1);
    jd.actuator.mode = ActuatorMode::PDTarget;
    jd.actuator.target = 0.0f;     // hold horizontal
    jd.actuator.kp = 300.0f;
    jd.actuator.kd = 30.0f;
    jd.actuator.maxTorque = 100.0f;
    const JointHandle j = w->createJoint(jd);
    TST_REQUIRE(j.valid());

    for (int i = 0; i < 480; ++i) w->step(1.0f / 120.0f);   // 4 s to settle

    const JointState s = w->jointState(j);
    std::printf("pd_hold: q = %.4f (target 0), qd = %.4f\n", s.q, s.qd);
    TST_REQUIRE(std::fabs(s.q) < 0.1f);     // holds near horizontal (small gravity sag)
    TST_REQUIRE(std::fabs(s.qd) < 0.2f);    // and is at rest
}

// Torque mode drives the joint in the commanded direction; q/qd reflect the motion.
TST_CASE(physics, unit, actuator_torque_moves_joint) {
    auto w = makeWorld(Vec3(0), 24, 2);   // no gravity: isolate the actuator
    const BodyHandle a = addBody(*w, BodyType::Static, Vec3(0), 0.0f);
    const BodyHandle b = addBody(*w, BodyType::Dynamic, Vec3(1, 0, 0), 1.0f, Vec3(0), Vec3(0), 0.5f);

    JointDef jd;
    jd.type = JointType::Revolute;
    jd.a = a; jd.b = b;
    jd.localAnchorA = Vec3(0, 0, 0);
    jd.localAnchorB = Vec3(-1, 0, 0);
    jd.localAxisA = Vec3(0, 0, 1);
    jd.localAxisB = Vec3(0, 0, 1);
    jd.actuator.mode = ActuatorMode::Torque;
    jd.actuator.torque = 5.0f;    // positive torque about +Z
    const JointHandle j = w->createJoint(jd);

    for (int i = 0; i < 120; ++i) w->step(1.0f / 120.0f);   // 1 s

    const JointState s = w->jointState(j);
    std::printf("torque: q = %.4f, qd = %.4f (expect both > 0)\n", s.q, s.qd);
    TST_REQUIRE(s.q > 0.2f);      // rotated in the +Z direction
    TST_REQUIRE(s.qd > 0.2f);     // and is still accelerating/moving +
}

// Bulk SoA action write + bulk q/qd read: three independent hinges get per-index torques; the
// bulk state readback reflects each joint's commanded direction (indexing is stable/consistent).
TST_CASE(physics, unit, actuator_bulk_write_read) {
    auto w = makeWorld(Vec3(0), 24, 2);
    JointHandle handles[3];
    for (int k = 0; k < 3; ++k) {
        // Separate the three pendulums along Z so their colliders never touch.
        const Vec3 base(0, 0, static_cast<float>(k) * 5.0f);
        const BodyHandle a = addBody(*w, BodyType::Static, base, 0.0f);
        const BodyHandle b = addBody(*w, BodyType::Dynamic, base + Vec3(1, 0, 0), 1.0f,
                                     Vec3(0), Vec3(0), 0.5f);
        JointDef jd;
        jd.type = JointType::Revolute;
        jd.a = a; jd.b = b;
        jd.localAnchorA = Vec3(0, 0, 0);
        jd.localAnchorB = Vec3(-1, 0, 0);
        jd.localAxisA = Vec3(0, 0, 1);
        jd.localAxisB = Vec3(0, 0, 1);
        jd.actuator.mode = ActuatorMode::Torque;   // torque commands supplied via bulk write
        handles[k] = w->createJoint(jd);
        TST_REQUIRE(handles[k].valid());
        TST_REQUIRE(handles[k].index == static_cast<uint32_t>(k));   // slot order == creation order
    }

    const Real torques[3] = { 4.0f, 0.0f, -4.0f };
    w->setJointTorques(std::span<const Real>(torques, 3));

    for (int i = 0; i < 120; ++i) w->step(1.0f / 120.0f);

    const std::span<const JointState> states = w->jointStates();
    TST_REQUIRE(states.size() == 3);
    std::printf("bulk: qd = [%.3f, %.3f, %.3f]\n", states[0].qd, states[1].qd, states[2].qd);
    TST_REQUIRE(states[0].qd > 0.2f);              // +torque → +rate
    TST_REQUIRE(std::fabs(states[1].qd) < 0.05f);  // zero torque → at rest
    TST_REQUIRE(states[2].qd < -0.2f);             // -torque → -rate

    // Single accessor agrees with the bulk readback.
    for (int k = 0; k < 3; ++k)
        TST_REQUIRE(std::fabs(w->jointState(handles[k]).qd - states[k].qd) < 1e-4f);
}

// Ball spherical PD: a 3-DOF ball actuator holds its child at a target relative orientation
// against gravity (a passive ball would let the offset body swing/rotate down).
TST_CASE(physics, unit, ball_actuator_holds_orientation) {
    auto w = makeWorld(Vec3(0, -9.81f, 0), 32, 4);
    const BodyHandle a = addBody(*w, BodyType::Static, Vec3(0), 0.0f);
    const BodyHandle b = addBody(*w, BodyType::Dynamic, Vec3(1, 0, 0), 1.0f, Vec3(0), Vec3(0), 0.5f);

    JointDef jd;
    jd.type = JointType::Ball;
    jd.a = a; jd.b = b;
    jd.localAnchorA = Vec3(0, 0, 0);
    jd.localAnchorB = Vec3(-1, 0, 0);   // COM offset 1 m from the pivot ⇒ gravity torque
    jd.actuator.mode = ActuatorMode::PDTarget;
    jd.actuator.ballTarget = Quat(1, 0, 0, 0);   // hold the neutral (identity) relative orientation
    jd.actuator.kp = 400.0f;
    jd.actuator.kd = 40.0f;
    jd.actuator.maxTorque = 300.0f;
    const JointHandle j = w->createJoint(jd);
    TST_REQUIRE(j.valid());

    for (int i = 0; i < 480; ++i) w->step(1.0f / 120.0f);   // 4 s

    const engine::Transform tb = w->pose(b);
    std::printf("ball_pd: qxyz = (%.4f, %.4f, %.4f), pos=(%.3f,%.3f,%.3f)\n",
                tb.rotation.x, tb.rotation.y, tb.rotation.z, tb.position.x, tb.position.y, tb.position.z);
    // Orientation held near identity (the 3-DOF PD resisted the gravity-induced rotation).
    TST_REQUIRE(std::fabs(tb.rotation.x) < 0.15f);
    TST_REQUIRE(std::fabs(tb.rotation.y) < 0.15f);
    TST_REQUIRE(std::fabs(tb.rotation.z) < 0.15f);
}
