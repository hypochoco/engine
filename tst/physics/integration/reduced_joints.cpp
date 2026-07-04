//
//  reduced_joints.cpp
//  engine::tst / physics / integration
//
//  Additional coverage for the reduced-coordinate (Featherstone) backend beyond the E0–E2 suite
//  in reduced.cpp: per-joint-type behavior (fixed weld, revolute torque/PD, off-axis hinge),
//  capsule contact, and — deliberately — cases that pin known gaps/bugs (joint limits and
//  WorldDef damping are silently ignored by this backend; ball-joint q/qd readout is 0). The
//  bug-pinning tests assert the CORRECT behavior, so they fail until the gap is fixed.
//

#include <cmath>
#include <cstdio>

#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics;

namespace {

constexpr Real kG = Real(9.81);

BodyHandle addSphere(PhysicsWorld& w, const Vec3& pos, Real mass, Real r, BodyType t) {
    BodyDef d; d.type = t; d.mass = mass;
    d.collider.type = ColliderDesc::Type::Sphere; d.collider.sphere.radius = r;
    d.position = pos; return w.createBody(d);
}

} // namespace

// --- Fixed (0-DOF) joint: a rigid weld to a static parent must hold its pose exactly ----------
TST_CASE(physics, integration, reduced_fixed_joint_weld) {
    WorldDef wd; wd.gravity = Vec3(0, -kG, 0); wd.substeps = 8;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef anchorDef; anchorDef.type = BodyType::Static;
    const BodyHandle anchor = w->createBody(anchorDef);
    const BodyHandle child = addSphere(*w, Vec3(1, 0, 0), 1.0f, 0.1f, BodyType::Dynamic);

    JointDef jd; jd.type = JointType::Fixed; jd.a = anchor; jd.b = child;
    jd.localAnchorA = Vec3(1, 0, 0);      // weld point in parent (static @ origin) frame
    jd.localAnchorB = Vec3(0, 0, 0);      // ...at the child's origin
    w->createJoint(jd);

    const Real dt = Real(1) / Real(240);
    for (int i = 0; i < 480; ++i) w->step(dt);   // 2 s under gravity
    const Vec3 p = w->pose(child).position;
    const Vec3 v = w->linearVelocities()[child.index];
    std::printf("reduced_fixed_weld: p=(%.4f,%.4f,%.4f) |v|=%.4e\n", p.x, p.y, p.z, glm::length(v));
    TST_REQUIRE(std::isfinite(p.y));
    TST_REQUIRE(glm::length(p - Vec3(1, 0, 0)) < 1e-3f);   // rigid weld: no droop
    TST_REQUIRE(glm::length(v) < 1e-3f);                   // and no motion
}

// --- Revolute Torque mode: a constant torque spins the hinge up (zero-g, fixed base) ----------
TST_CASE(physics, integration, reduced_revolute_torque_spins) {
    WorldDef wd; wd.gravity = Vec3(0); wd.substeps = 8;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef anchorDef; anchorDef.type = BodyType::Static;
    const BodyHandle anchor = w->createBody(anchorDef);
    const BodyHandle bob = addSphere(*w, Vec3(1, 0, 0), 1.0f, 0.05f, BodyType::Dynamic);

    JointDef jd; jd.type = JointType::Revolute; jd.a = anchor; jd.b = bob;
    jd.localAnchorA = Vec3(0); jd.localAnchorB = Vec3(-1, 0, 0);
    jd.localAxisA = Vec3(0, 0, 1); jd.localAxisB = Vec3(0, 0, 1);
    const JointHandle jh = w->createJoint(jd);

    Actuator act; act.mode = ActuatorMode::Torque; act.torque = 1.0f; act.maxTorque = 10.0f;
    w->setJointActuator(jh, act);

    const Real dt = Real(1) / Real(240);
    for (int i = 0; i < 120; ++i) w->step(dt);   // 0.5 s
    const Real q = w->jointState(jh).q, qd = w->jointState(jh).qd;
    std::printf("reduced_revolute_torque: q=%.4f qd=%.4f\n", q, qd);
    TST_REQUIRE(q > 0.05f);    // +z torque ⇒ q increases
    TST_REQUIRE(qd > 0.2f);    // and it is spinning up
}

// --- Revolute PDTarget: PD servo drives the hinge to a nonzero target angle (zero-g) ----------
TST_CASE(physics, integration, reduced_revolute_pd_reaches_target) {
    WorldDef wd; wd.gravity = Vec3(0); wd.substeps = 8;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef anchorDef; anchorDef.type = BodyType::Static;
    const BodyHandle anchor = w->createBody(anchorDef);
    const BodyHandle bob = addSphere(*w, Vec3(1, 0, 0), 1.0f, 0.05f, BodyType::Dynamic);

    JointDef jd; jd.type = JointType::Revolute; jd.a = anchor; jd.b = bob;
    jd.localAnchorA = Vec3(0); jd.localAnchorB = Vec3(-1, 0, 0);
    jd.localAxisA = Vec3(0, 0, 1); jd.localAxisB = Vec3(0, 0, 1);
    const JointHandle jh = w->createJoint(jd);

    Actuator act; act.mode = ActuatorMode::PDTarget; act.target = 0.8f; act.kp = 60.0f; act.kd = 8.0f; act.maxTorque = 100.0f;
    w->setJointActuator(jh, act);

    const Real dt = Real(1) / Real(240);
    for (int i = 0; i < 720; ++i) w->step(dt);   // 3 s to converge
    const Real q = w->jointState(jh).q;
    std::printf("reduced_revolute_pd: q=%.4f (target 0.8)\n", q);
    TST_REQUIRE(std::fabs(q - 0.8f) < 0.03f);
}

// --- Off-axis hinge: a revolute about world-X swings in the y–z plane, staying out of x -------
TST_CASE(physics, integration, reduced_offaxis_hinge_swings) {
    const Real L = 1.0f;
    WorldDef wd; wd.gravity = Vec3(0, -kG, 0); wd.substeps = 16;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef anchorDef; anchorDef.type = BodyType::Static;
    const BodyHandle anchor = w->createBody(anchorDef);

    // Bob built 36.9° off straight-down in the y–z plane (|com| = L). Hinge axis = world +x.
    const Vec3 com(0, -0.8f * L, 0.6f * L);
    const BodyHandle bob = addSphere(*w, com, 1.0f, 0.05f, BodyType::Dynamic);

    JointDef jd; jd.type = JointType::Revolute; jd.a = anchor; jd.b = bob;
    jd.localAnchorA = Vec3(0); jd.localAnchorB = -com;
    jd.localAxisA = Vec3(1, 0, 0); jd.localAxisB = Vec3(1, 0, 0);
    w->createJoint(jd);

    const Real dt = Real(1) / Real(240);
    Real minY = com.y, maxAbsX = 0, minZ = com.z;
    for (int i = 0; i < 480; ++i) {              // 2 s
        w->step(dt);
        const Vec3 p = w->pose(bob).position;
        minY = std::min(minY, p.y); maxAbsX = std::max(maxAbsX, std::fabs(p.x)); minZ = std::min(minZ, p.z);
    }
    std::printf("reduced_offaxis_hinge: minY=%.4f maxAbsX=%.4e minZ=%.4f\n", minY, maxAbsX, minZ);
    TST_REQUIRE(minY < -0.98f * L);     // swings down through the bottom (0,-L,0)
    TST_REQUIRE(minZ < -0.4f * L);      // and up the far side (z goes negative)
    TST_REQUIRE(maxAbsX < 0.02f);       // stays in the y–z plane (correct hinge axis)
}

// --- Capsule contact: a horizontal capsule settles on the plane at its radius -----------------
TST_CASE(physics, integration, reduced_capsule_rests_on_plane) {
    WorldDef wd; wd.gravity = Vec3(0, -kG, 0); wd.substeps = 12;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef ground; ground.type = BodyType::Static;
    ground.collider.type = ColliderDesc::Type::Plane;
    ground.collider.plane = Plane{ Vec3(0, 1, 0), Real(0) };
    ground.material.friction = 0.8f;
    w->createBody(ground);

    const Real r = 0.15f, hh = 0.3f;
    BodyDef cap; cap.type = BodyType::Dynamic; cap.mass = 1.0f;
    cap.collider.type = ColliderDesc::Type::Capsule; cap.collider.capsule.radius = r; cap.collider.capsule.halfHeight = hh;
    cap.position = Vec3(0, 0.8f, 0);
    cap.orientation = glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1));   // lay it horizontal (axis → x)
    cap.material.friction = 0.8f;
    const BodyHandle bh = w->createBody(cap);

    const Real dt = Real(1) / Real(240);
    Real minY = 1e9f;
    for (int i = 0; i < 720; ++i) { w->step(dt); minY = std::min(minY, w->pose(bh).position.y); }
    const Real finalY = w->pose(bh).position.y;
    const Real finalSpeed = glm::length(w->linearVelocities()[bh.index]);
    std::printf("reduced_capsule_rests: finalY=%.4f (r=%.2f) minY=%.4f speed=%.4e\n", finalY, r, minY, finalSpeed);
    TST_REQUIRE(std::fabs(finalY - r) < 0.02f);   // rests on its side at the radius
    TST_REQUIRE(minY > r - 0.03f);                // no meaningful penetration
    TST_REQUIRE(finalSpeed < 0.05f);              // settled
}

// --- BUG PIN: joint limits are silently ignored by the reduced backend ------------------------
// Mirrors the realtime `hinge_limit_stops` test. A limited hinge should stop at the clamp; the
// reduced backend never reads enableLimit/lowerLimit/upperLimit, so the pendulum swings past.
TST_CASE(physics, integration, reduced_hinge_limit_stops) {
    WorldDef wd; wd.gravity = Vec3(0, -kG, 0); wd.substeps = 16;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef anchorDef; anchorDef.type = BodyType::Static;
    const BodyHandle anchor = w->createBody(anchorDef);
    const BodyHandle bob = addSphere(*w, Vec3(1, 0, 0), 1.0f, 0.05f, BodyType::Dynamic);

    JointDef jd; jd.type = JointType::Revolute; jd.a = anchor; jd.b = bob;
    jd.localAnchorA = Vec3(0); jd.localAnchorB = Vec3(-1, 0, 0);
    jd.localAxisA = Vec3(0, 0, 1); jd.localAxisB = Vec3(0, 0, 1);
    jd.enableLimit = true; jd.lowerLimit = -0.5f; jd.upperLimit = 1.0f;   // clamp the downswing at -0.5 rad
    const JointHandle jh = w->createJoint(jd);

    const Real dt = Real(1) / Real(240);
    Real minQ = 0;
    for (int i = 0; i < 720; ++i) { w->step(dt); minQ = std::min(minQ, w->jointState(jh).q); }
    const Real q = w->jointState(jh).q;
    std::printf("reduced_hinge_limit: settled q=%.4f minQ=%.4f (clamp -0.5, free-hang ~-1.57)\n", q, minQ);
    TST_REQUIRE(minQ > -0.60f);   // must not blow past the clamp (FAILS: limits ignored)
}

// --- BUG PIN: WorldDef damping is not applied to the floating base ----------------------------
// linearDamping is read by the realtime backend + physics_env, but the reduced ctor ignores it,
// and angularDamping is applied only to joint DOFs (not the floating base twist). A lone floating
// body with damping and no gravity should slow down; here it coasts forever.
TST_CASE(physics, integration, reduced_floating_body_damping_decays) {
    WorldDef wd; wd.gravity = Vec3(0); wd.substeps = 8;
    wd.linearDamping = 2.0f; wd.angularDamping = 2.0f;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    const BodyHandle h = addSphere(*w, Vec3(0), 1.0f, 0.2f, BodyType::Dynamic);
    w->setBodyState(h, Vec3(0), Quat(1, 0, 0, 0), Vec3(1, 0, 0), Vec3(0, 3, 0));
    w->refreshState();
    const Real v0 = glm::length(w->linearVelocities()[h.index]);
    const Real w0 = glm::length(w->angularVelocities()[h.index]);

    const Real dt = Real(1) / Real(240);
    for (int i = 0; i < 480; ++i) w->step(dt);   // 2 s; damping ~2/s ⇒ e^-4 decay expected
    const Real v1 = glm::length(w->linearVelocities()[h.index]);
    const Real w1 = glm::length(w->angularVelocities()[h.index]);
    std::printf("reduced_floating_damping: v %.4f→%.4f  w %.4f→%.4f\n", v0, v1, w0, w1);
    TST_REQUIRE(v1 < 0.5f * v0);   // linear velocity should decay (FAILS: linearDamping ignored)
    TST_REQUIRE(w1 < 0.5f * w0);   // angular velocity should decay (FAILS: base twist undamped)
}

// --- Ball-joint multi-DOF readout: rotation + angular velocity are reported (q/qd stay 0) --------
TST_CASE(physics, integration, reduced_ball_state_readout) {
    WorldDef wd; wd.gravity = Vec3(0, -kG, 0); wd.substeps = 16;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef anchorDef; anchorDef.type = BodyType::Static;
    const BodyHandle anchor = w->createBody(anchorDef);
    const Vec3 com(0.6f, -0.8f, 0);
    const BodyHandle bob = addSphere(*w, com, 1.0f, 0.05f, BodyType::Dynamic);
    JointDef jd; jd.type = JointType::Ball; jd.a = anchor; jd.b = bob;
    jd.localAnchorA = Vec3(0); jd.localAnchorB = -com;
    const JointHandle jh = w->createJoint(jd);

    const Real dt = Real(1) / Real(240);
    for (int i = 0; i < 240; ++i) w->step(dt);   // swings ⇒ body clearly moves
    const bool moved = glm::length(w->angularVelocities()[bob.index]) > 0.1f;
    const JointState js = w->jointState(jh);
    const Real rotMag = glm::length(js.rotation), wMag = glm::length(js.angularVelocity);
    std::printf("reduced_ball_readout: moved=%d |rotvec|=%.4f |angVel|=%.4f q=%.4f qd=%.4f\n",
                moved, rotMag, wMag, js.q, js.qd);
    TST_REQUIRE(moved);                    // the ball joint genuinely articulates
    TST_REQUIRE(rotMag > 0.1f);            // ...and its orientation is now readable (multi-DOF)
    TST_REQUIRE(wMag > 0.1f);              // ...and its angular velocity too
    TST_REQUIRE(js.q == Real(0));          // scalar q/qd remain revolute-only (0 for ball)
    TST_REQUIRE(js.qd == Real(0));
}
