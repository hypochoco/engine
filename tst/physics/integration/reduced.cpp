//
//  reduced.cpp
//  engine::tst / physics / integration
//
//  Phase E0c: validate the reduced-coordinate (Featherstone/ABA) backend on classic articulated
//  problems where the physics is analytically known — a single pendulum's small-angle period and
//  a passive double pendulum's energy conservation. These need no contacts and exercise the ABA
//  core, the gravity term, and forward kinematics.
//

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics;

namespace {

constexpr Real kG = Real(9.81);

// A revolute pendulum: static pivot at the origin + a spherical bob whose COM sits at `com`
// (world). q = 0 is the built configuration; the hinge axis is +z (swings in the x–y plane).
JointHandle addPendulumLink(PhysicsWorld& w, BodyHandle pivot, const Vec3& com, Real mass,
                            Real radius, BodyHandle& outBob) {
    BodyDef bob;
    bob.type = BodyType::Dynamic;
    bob.mass = mass;
    bob.collider.type = ColliderDesc::Type::Sphere;
    bob.collider.sphere.radius = radius;
    bob.position = com;
    outBob = w.createBody(bob);

    JointDef jd;
    jd.type = JointType::Revolute;
    jd.a = pivot; jd.b = outBob;
    jd.localAnchorA = Vec3(0);          // pivot at the parent origin (set by caller's pivot pose)
    jd.localAnchorB = -com;             // ...expressed in the bob frame (identity orientation)
    jd.localAxisA = Vec3(0, 0, 1);
    jd.localAxisB = Vec3(0, 0, 1);
    return w.createJoint(jd);
}

} // namespace

TST_CASE(physics, integration, reduced_pendulum_period) {
    const Real L = 1.0f, r = 0.03f, m = 1.0f, theta0 = 0.12f;   // small angle from straight-down

    WorldDef wd;
    wd.gravity = Vec3(0, -kG, 0);
    wd.substeps = 8;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef anchorDef; anchorDef.type = BodyType::Static;
    const BodyHandle anchor = w->createBody(anchorDef);

    // Bob built at theta0 from straight-down ⇒ oscillates about the down equilibrium (q=-theta0),
    // released from rest at the q=0 turning point.
    const Vec3 com(std::sin(theta0) * L, -std::cos(theta0) * L, 0);
    BodyHandle bob;
    const JointHandle jh = addPendulumLink(*w, anchor, com, m, r, bob);

    const Real dt = Real(1) / Real(240);
    Real prevQ = 0;
    Real measuredPeriod = -1;
    Real minQ = 0;
    for (int i = 1; i <= 1200; ++i) {                 // up to 5 s
        w->step(dt);
        const Real q = w->jointState(jh).q;
        minQ = std::min(minQ, q);
        // First return to the top (q crosses back up to ~0 from below) marks one full period.
        if (measuredPeriod < 0 && i > 20 && prevQ < Real(-1e-4) && q >= Real(-1e-4))
            measuredPeriod = i * dt;
        prevQ = q;
    }

    const Real expected = Real(2) * Real(M_PI) * std::sqrt(L / kG);   // ~2.006 s
    std::printf("reduced_pendulum: minQ=%.4f (expect ~%.3f) measuredT=%.4f expectedT=%.4f\n",
                minQ, -2 * theta0, measuredPeriod, expected);

    // Gravity sign: the bob must swing DOWN toward equilibrium (q goes negative, ~ -2*theta0).
    TST_REQUIRE(minQ < -theta0 * 0.8f);
    TST_REQUIRE(measuredPeriod > 0);
    TST_REQUIRE(std::fabs(measuredPeriod - expected) / expected < 0.03f);   // within 3%
}

TST_CASE(physics, integration, reduced_double_pendulum_energy) {
    const Real m = 1.0f, r = 0.05f;

    WorldDef wd;
    wd.gravity = Vec3(0, -kG, 0);
    wd.substeps = 16;                        // finer substeps → less integrator energy drift
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef anchorDef; anchorDef.type = BodyType::Static;
    const BodyHandle anchor = w->createBody(anchorDef);

    // Two links, both built horizontal (+x) — a large-swing, chaotic double pendulum.
    BodyHandle b1, b2;
    addPendulumLink(*w, anchor, Vec3(1, 0, 0), m, r, b1);   // COM 1 m out
    // link 2 pivots at the end of link 1 (world (2,0,0)); its COM at (2,0,0), joint anchor there.
    {
        BodyDef bob; bob.type = BodyType::Dynamic; bob.mass = m;
        bob.collider.type = ColliderDesc::Type::Sphere; bob.collider.sphere.radius = r;
        bob.position = Vec3(2, 0, 0);
        b2 = w->createBody(bob);
        JointDef jd; jd.type = JointType::Revolute; jd.a = b1; jd.b = b2;
        jd.localAnchorA = Vec3(1, 0, 0);     // end of link 1 in its frame (COM at origin)
        jd.localAnchorB = Vec3(-1, 0, 0);    // that joint in link 2's frame
        jd.localAxisA = Vec3(0, 0, 1); jd.localAxisB = Vec3(0, 0, 1);
        w->createJoint(jd);
    }

    const Real Ic = Real(0.4) * m * r * r;   // solid-sphere inertia about COM (diagonal)
    auto totalEnergy = [&]() {
        const auto P = w->poses();
        const auto LV = w->linearVelocities();
        const auto AV = w->angularVelocities();
        Real E = 0;
        for (BodyHandle h : { b1, b2 }) {
            const Vec3 v = LV[h.index], wv = AV[h.index];
            E += Real(0.5) * m * glm::dot(v, v) + Real(0.5) * Ic * glm::dot(wv, wv);   // KE
            E += m * kG * P[h.index].position.y;                                       // PE
        }
        return E;
    };

    w->refreshState();
    const Real E0 = totalEnergy();
    auto potentialEnergy = [&]() {
        const auto P = w->poses();
        return m * kG * (P[b1.index].position.y + P[b2.index].position.y);
    };
    Real maxDrift = 0, peMin = potentialEnergy(), peMax = peMin;
    const Real dt = Real(1) / Real(240);
    for (int i = 0; i < 1200; ++i) {         // 5 s
        w->step(dt);
        maxDrift = std::max(maxDrift, std::fabs(totalEnergy() - E0));
        const Real pe = potentialEnergy();
        peMin = std::min(peMin, pe); peMax = std::max(peMax, pe);
    }
    // Scale drift by the characteristic energy that actually sloshes (PE range), since E0≈0 here.
    const Real scale = std::max(Real(1), peMax - peMin);
    std::printf("reduced_double_pendulum: E0=%.4f maxDrift=%.4f energyScale=%.4f (%.2f%%)\n",
                E0, maxDrift, scale, 100.0f * maxDrift / scale);
    // Semi-implicit Euler conserves energy to a bounded oscillation, not exactly — small % over 5 s.
    TST_REQUIRE(maxDrift / scale < 0.02f);
}

TST_CASE(physics, integration, reduced_floating_momentum) {
    // A free-floating 3-link chain with NO gravity. Internal joint torques cannot change total
    // linear/angular momentum, so both must be conserved and the COM must travel in a straight
    // line at constant velocity — the defining property of a correct floating base.
    WorldDef wd;
    wd.gravity = Vec3(0);                     // free space
    wd.substeps = 8;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    struct Seg { BodyHandle h; Real m; Real r; };
    auto sphere = [&](const Vec3& pos, Real m, Real r, BodyType t) {
        BodyDef d; d.type = t; d.mass = m;
        d.collider.type = ColliderDesc::Type::Sphere; d.collider.sphere.radius = r;
        d.position = pos; return w->createBody(d);
    };
    // Floating root (Dynamic) + two revolute links; link 2 is bent 90° (up +y) so that a spin
    // about z produces a tangential centrifugal load on it → the z-hinges actually articulate.
    Seg s0{ sphere(Vec3(0, 0, 0), 2.0f, 0.20f, BodyType::Dynamic), 2.0f, 0.20f };
    Seg s1{ sphere(Vec3(1, 0, 0), 1.0f, 0.15f, BodyType::Dynamic), 1.0f, 0.15f };
    Seg s2{ sphere(Vec3(1.5f, 0.5f, 0), 1.0f, 0.15f, BodyType::Dynamic), 1.0f, 0.15f };

    auto hinge = [&](BodyHandle a, BodyHandle b, const Vec3& anchorA, const Vec3& anchorB) {
        JointDef jd; jd.type = JointType::Revolute; jd.a = a; jd.b = b;
        jd.localAnchorA = anchorA; jd.localAnchorB = anchorB;
        jd.localAxisA = Vec3(0, 0, 1); jd.localAxisB = Vec3(0, 0, 1);
        return w->createJoint(jd);
    };
    const JointHandle j1 = hinge(s0.h, s1.h, Vec3(0.5f, 0, 0), Vec3(-0.5f, 0, 0));
    const JointHandle j2 = hinge(s1.h, s2.h, Vec3(0.5f, 0, 0), Vec3(0, -0.5f, 0));

    // Give the base an initial twist (translation + spin). With offset links the spin drives the
    // joints through Coriolis/centrifugal coupling, so the chain genuinely articulates while all
    // forces stay internal (no gravity, no actuator) ⇒ momentum must be conserved.
    w->setBodyState(s0.h, Vec3(0), Quat(1, 0, 0, 0), Vec3(0.5f, 0.3f, 0.1f), Vec3(0, 0, 0.8f));
    w->refreshState();

    const std::array<Seg, 3> segs{ s0, s1, s2 };
    Real Mtot = 0; for (const Seg& s : segs) Mtot += s.m;

    auto momentum = [&](Vec3& P, Vec3& Lang) {
        const auto pos = w->poses();
        const auto lv = w->linearVelocities();
        const auto av = w->angularVelocities();
        P = Vec3(0); Lang = Vec3(0);
        for (const Seg& s : segs) {
            const Vec3 v = lv[s.h.index], wv = av[s.h.index], p = pos[s.h.index].position;
            const Real Ic = Real(0.4) * s.m * s.r * s.r;     // isotropic ⇒ rotation-invariant
            P += s.m * v;
            Lang += glm::cross(p, s.m * v) + Ic * wv;         // angular momentum about the origin
        }
    };

    Vec3 P0, L0; momentum(P0, L0);
    const Vec3 comVel0 = P0 / Mtot;

    const Real dt = Real(1) / Real(240);
    Real maxdP = 0, maxdL = 0, maxJointMove = 0;
    for (int i = 0; i < 480; ++i) {                 // 2 s
        w->step(dt);
        Vec3 P, L; momentum(P, L);
        maxdP = std::max(maxdP, glm::length(P - P0));
        maxdL = std::max(maxdL, glm::length(L - L0));
        maxJointMove = std::max({ maxJointMove, std::fabs(w->jointState(j1).q), std::fabs(w->jointState(j2).q) });
    }
    std::printf("reduced_floating_momentum: |P0|=%.4f |L0|=%.4f maxdP=%.3e maxdL=%.3e jointMove=%.3f comVel=(%.3f,%.3f,%.3f)\n",
                glm::length(P0), glm::length(L0), maxdP, maxdL, maxJointMove, comVel0.x, comVel0.y, comVel0.z);

    TST_REQUIRE(glm::length(P0) > 0.1f);                       // there is real motion to conserve
    TST_REQUIRE(maxJointMove > 0.05f);                         // the chain genuinely articulates
    TST_REQUIRE(maxdP / glm::length(P0) < 2e-3f);              // linear momentum conserved
    TST_REQUIRE(maxdL / std::max(0.1f, glm::length(L0)) < 5e-3f);   // angular momentum conserved
}

TST_CASE(physics, integration, reduced_sphere_rests_on_plane) {
    // E1: a floating dynamic sphere falls under gravity and settles on a static ground plane,
    // resting at its radius without penetrating and coming (nearly) to rest — the contact solve.
    WorldDef wd;
    wd.gravity = Vec3(0, -kG, 0);
    wd.substeps = 8;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    BodyDef ground;
    ground.type = BodyType::Static;
    ground.collider.type = ColliderDesc::Type::Plane;
    ground.collider.plane = Plane{ Vec3(0, 1, 0), Real(0) };
    ground.material.friction = 0.8f;
    w->createBody(ground);

    const Real r = 0.2f;
    BodyDef ball;
    ball.type = BodyType::Dynamic; ball.mass = 1.0f;
    ball.collider.type = ColliderDesc::Type::Sphere; ball.collider.sphere.radius = r;
    ball.position = Vec3(0, 1.0f, 0);
    ball.material.friction = 0.8f;
    const BodyHandle bh = w->createBody(ball);

    const Real dt = Real(1) / Real(240);
    Real minY = 1e9f;
    for (int i = 0; i < 720; ++i) {                 // 3 s
        w->step(dt);
        minY = std::min(minY, w->pose(bh).position.y);
    }
    const Real finalY = w->pose(bh).position.y;
    const Real finalVy = w->linearVelocities()[bh.index].y;
    std::printf("reduced_sphere_rests: finalY=%.4f (r=%.2f) minY=%.4f finalVy=%.4e\n",
                finalY, r, minY, finalVy);

    TST_REQUIRE(std::fabs(finalY - r) < 0.01f);      // rests at the radius
    TST_REQUIRE(minY > r - 0.02f);                   // never penetrated meaningfully
    TST_REQUIRE(std::fabs(finalVy) < 0.05f);         // settled to rest
}

TST_CASE(physics, integration, reduced_box_friction_holds_on_slope) {
    // E1 friction: a box on a tilted plane holds (high friction) rather than sliding away.
    WorldDef wd;
    wd.gravity = Vec3(0, -kG, 0);
    wd.substeps = 12;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    // Ground tilted ~17° about z: normal rotated from +y toward +x.
    const Real tilt = 0.30f;                         // radians (~17°, tan≈0.31)
    BodyDef ground;
    ground.type = BodyType::Static;
    ground.collider.type = ColliderDesc::Type::Plane;
    ground.collider.plane = Plane{ glm::normalize(Vec3(std::sin(tilt), std::cos(tilt), 0)), Real(0) };
    ground.material.friction = 1.0f;                 // μ=1 > tan(17°)=0.31 ⇒ holds
    w->createBody(ground);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box; box.collider.box.halfExtents = Vec3(0.15f);
    box.position = Vec3(0, 0.5f, 0);
    box.material.friction = 1.0f;
    const BodyHandle bh = w->createBody(box);

    const Real dt = Real(1) / Real(240);
    for (int i = 0; i < 240; ++i) w->step(dt);       // settle onto the slope
    const Vec3 settled = w->pose(bh).position;
    for (int i = 0; i < 480; ++i) w->step(dt);       // 2 s more — should NOT slide down the slope
    const Vec3 finalP = w->pose(bh).position;

    const Real drift = glm::length(finalP - settled);
    std::printf("reduced_box_friction: settled=(%.3f,%.3f) final=(%.3f,%.3f) drift=%.4f\n",
                settled.x, settled.y, finalP.x, finalP.y, drift);
    TST_REQUIRE(drift < 0.03f);                      // static friction holds it in place
}

TST_CASE(physics, integration, reduced_box_slides_when_slippery) {
    // Complement to the hold test: with μ below tan(slope) the box must slide DOWN the incline
    // (down-slope is −x here) — proving the friction cone is doing real work, not numerical sticking.
    WorldDef wd;
    wd.gravity = Vec3(0, -kG, 0);
    wd.substeps = 12;
    auto w = createPhysicsWorld(Backend::Reduced, wd);

    const Real tilt = 0.30f;
    BodyDef ground;
    ground.type = BodyType::Static;
    ground.collider.type = ColliderDesc::Type::Plane;
    ground.collider.plane = Plane{ glm::normalize(Vec3(std::sin(tilt), std::cos(tilt), 0)), Real(0) };
    ground.material.friction = 0.05f;                // μ=0.05 < tan(17°)=0.31 ⇒ slides
    w->createBody(ground);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box; box.collider.box.halfExtents = Vec3(0.15f);
    box.position = Vec3(0, 0.5f, 0);
    box.material.friction = 0.05f;
    const BodyHandle bh = w->createBody(box);

    const Real dt = Real(1) / Real(240);
    for (int i = 0; i < 120; ++i) w->step(dt);       // land on the slope
    const Vec3 settled = w->pose(bh).position;
    for (int i = 0; i < 480; ++i) w->step(dt);       // 2 s — should slide down (−x, −y)
    const Vec3 finalP = w->pose(bh).position;

    std::printf("reduced_box_slides: settled=(%.3f,%.3f) final=(%.3f,%.3f) dx=%.3f\n",
                settled.x, settled.y, finalP.x, finalP.y, finalP.x - settled.x);
    // Normal leans toward +x ⇒ steepest descent (down-slope) is +x and downward.
    TST_REQUIRE(finalP.x > settled.x + 0.1f);        // slid down-slope (+x)
    TST_REQUIRE(finalP.y < settled.y - 0.02f);       // and downhill (lower)
}
