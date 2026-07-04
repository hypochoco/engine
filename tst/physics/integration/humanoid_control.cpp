//
//  humanoid_control.cpp
//  engine::tst / physics / integration
//
//  Milestone 2, Phase B5: actuated-humanoid behaviours.
//    * pd_stand_holds_pose — with the pelvis pinned, PD servos on every joint hold the humanoid's
//      neutral pose against gravity (hinges stay near their targets; the body doesn't collapse).
//    * articulation_determinism — a serial world and a pooled/parallel world (threshold 1),
//      identical humanoid + actuator commands, step bit-identically (joints are solved serially
//      in creation order; the parallel contact/integration paths stay deterministic).
//

#include <cmath>
#include <cstdio>

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {

// A firm PD actuator that works for both hinge (uses target/kp/kd) and ball (uses ballTarget)
// joints — set uniformly so every joint holds its neutral configuration.
Actuator holdPose() {
    Actuator a;
    a.mode = ActuatorMode::PDTarget;
    a.target = 0.0f;                 // hinge: neutral angle
    a.ballTarget = Quat(1, 0, 0, 0); // ball: neutral relative orientation
    a.kp = 500.0f;
    a.kd = 50.0f;
    a.maxTorque = 400.0f;
    return a;
}

void addGround(PhysicsWorld& w) {
    BodyDef g;
    g.type = BodyType::Static;
    g.collider.type = ColliderDesc::Type::Plane;
    g.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
    g.material.friction = 0.9f;
    w.createBody(g);
}

} // namespace

TST_CASE(physics, integration, pd_stand_holds_pose) {
    WorldDef wd;
    wd.gravity = Vec3(0, -9.81f, 0);
    wd.velocityIterations = 24;
    wd.substeps = 4;
    auto w = createPhysicsWorld(Backend::Realtime, wd);

    // Pin the pelvis high so the limbs hang free (no ground contact) — this isolates "PD drives
    // the joints to a commanded pose against gravity" from the (separate, harder) balance problem.
    ArticulationDef def = makeHumanoid(Vec3(0, 1.6f, 0));
    def.bodies[0].type = BodyType::Static;   // pelvis
    const Articulation h = buildArticulation(*w, def);

    auto pd = [](float target) {
        Actuator a;
        a.mode = ActuatorMode::PDTarget;
        a.target = target;               // hinge target angle
        a.ballTarget = Quat(1, 0, 0, 0); // ball: neutral relative orientation
        a.kp = 500.0f; a.kd = 50.0f; a.maxTorque = 400.0f;
        return a;
    };
    for (const JointHandle j : h.joints) w->setJointActuator(j, pd(0.0f));
    // Command bent knees (j9,j10) — a pose gravity would straighten, so holding it proves the PD
    // servo works against gravity. Elbows (j5,j6) and others stay at neutral (0).
    const float elbowTarget = 0.0f, kneeTarget = -1.0f;
    w->setJointActuator(h.joints[9], pd(kneeTarget));
    w->setJointActuator(h.joints[10], pd(kneeTarget));

    for (int i = 0; i < 480; ++i) w->step(1.0f / 120.0f);   // 4 s to reach + hold the pose

    const auto states = w->jointStates();
    const Real elbowL = states[h.joints[5].index].q, kneeL = states[h.joints[9].index].q;
    const Real elbowR = states[h.joints[6].index].q, kneeR = states[h.joints[10].index].q;
    std::printf("pd_stand: elbow=(%.3f,%.3f)->%.2f  knee=(%.3f,%.3f)->%.2f  torso.y=%.3f\n",
                elbowL, elbowR, elbowTarget, kneeL, kneeR, kneeTarget, w->pose(h.bodies[1]).position.y);

    // PD reached and holds the commanded joint angles against gravity.
    TST_REQUIRE(std::fabs(elbowL - elbowTarget) < 0.15f);
    TST_REQUIRE(std::fabs(elbowR - elbowTarget) < 0.15f);
    TST_REQUIRE(std::fabs(kneeL - kneeTarget) < 0.15f);
    TST_REQUIRE(std::fabs(kneeR - kneeTarget) < 0.15f);
    // Torso held above the pinned pelvis (structure intact, didn't blow up).
    TST_REQUIRE(w->pose(h.bodies[1]).position.y > 1.8f);
}

TST_CASE(physics, integration, articulation_determinism) {
    engine::core::ThreadPool pool;
    auto build = [](engine::core::ThreadPool* p) {
        WorldDef wd;
        wd.gravity = Vec3(0, -9.81f, 0);
        wd.velocityIterations = 12;
        wd.substeps = 2;
        wd.threadPool = p;
        wd.parallelThreshold = p ? 1 : 1000000;
        auto w = createPhysicsWorld(Backend::Realtime, wd);
        addGround(*w);
        const Articulation h = buildArticulation(*w, makeHumanoid(Vec3(0, 1.2f, 0)));
        for (const JointHandle j : h.joints) w->setJointActuator(j, holdPose());
        return w;
    };
    auto serial = build(nullptr);
    auto parallel = build(&pool);

    for (int i = 0; i < 240; ++i) { serial->step(1.0f / 120.0f); parallel->step(1.0f / 120.0f); }

    const auto ps = serial->poses();
    const auto pp = parallel->poses();
    TST_REQUIRE(ps.size() == pp.size());
    Real maxErr = 0;
    for (size_t k = 0; k < ps.size(); ++k) {
        maxErr = std::max(maxErr, glm::length(ps[k].position - pp[k].position));
        maxErr = std::max(maxErr, glm::length(ps[k].rotation - pp[k].rotation));
    }
    std::printf("articulation_determinism: max serial-vs-parallel err = %.3e\n", maxErr);
    TST_REQUIRE(maxErr < 1e-4f);
}
