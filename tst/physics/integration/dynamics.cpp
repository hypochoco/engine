//
//  dynamics.cpp
//  engine::tst / physics / integration
//
//  Complex, whole-world dynamics that exercise the solver end to end: restitution (energy
//  return), Coulomb friction (slide vs damp), a head-on elastic collision (impulse/velocity
//  exchange), a taller stack (solver stability), broadphase equivalence (SAP == grid), and
//  kinematic-body motion. Expected values come from closed-form mechanics; tolerances account
//  for the discrete sequential-impulse solver.
//
//  Backend behaviours these rely on (see sequential_impulse_world.cpp):
//    * restitution combines via min(eA,eB) and only activates above a 1 m/s approach speed,
//    * friction combines via sqrt(muA*muB).
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {

std::unique_ptr<PhysicsWorld> makeWorld(Vec3 gravity, int velIters, int substeps,
                                        BroadphaseKind bp = BroadphaseKind::UniformGrid) {
    WorldDef wd;
    wd.gravity = gravity;
    wd.velocityIterations = velIters;
    wd.substeps = substeps;
    wd.broadphase = bp;
    return createPhysicsWorld(Backend::Realtime, wd);
}

BodyHandle addGroundPlane(PhysicsWorld& w, Real friction) {
    BodyDef d;
    d.type = BodyType::Static;
    d.collider.type = ColliderDesc::Type::Plane;
    d.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
    d.material.friction = friction;
    return w.createBody(d);
}

// Drop a sphere from center-y=3 onto the ground, return the peak center-y reached AFTER the
// first contact — i.e. how high it bounces back. Restitution combines via min(eA,eB), so BOTH
// bodies get the same coefficient.
Real bounceHeight(Real restitution, bool ccd = true) {
    WorldDef wd;
    wd.gravity = Vec3(0, -9.81f, 0);
    wd.velocityIterations = 8;
    wd.substeps = 2;
    wd.continuousDetection = ccd;
    auto w = createPhysicsWorld(Backend::Realtime, wd);
    {
        BodyDef ground;
        ground.type = BodyType::Static;
        ground.collider.type = ColliderDesc::Type::Plane;
        ground.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
        ground.material.friction = 0.0f;
        ground.material.restitution = restitution;
        w->createBody(ground);
    }
    BodyDef s;
    s.type = BodyType::Dynamic; s.mass = 1.0f;
    s.collider.type = ColliderDesc::Type::Sphere;
    s.collider.sphere = Sphere{ 0.5f };
    s.material.restitution = restitution;
    s.material.friction = 0.0f;
    s.position = Vec3(0, 3.0f, 0);
    const BodyHandle h = w->createBody(s);

    bool contacted = false;
    Real peak = 0;
    for (int i = 0; i < 400; ++i) {
        w->step(1.0f / 120.0f);
        const Real y = w->pose(h).position.y;
        if (y < 0.6f) contacted = true;          // reached the ground
        if (contacted) peak = std::max(peak, y);  // highest point after the first contact
    }
    return peak;
}

} // namespace

// Restitution: e=1 returns most of the drop height; e=0 does not bounce. Impact speed here is
// ~7 m/s (well above the solver's 1 m/s restitution threshold).
// Restitution: e=1 returns essentially all of the drop height and e=0 does not bounce (impact
// speed ~7 m/s, above the solver's 1 m/s threshold). Checked with continuous detection BOTH on
// (the WorldDef default) and off — a speculative contact must not suppress the bounce.
TST_CASE(physics, integration, restitution) {
    for (bool ccd : { false, true }) {
        const Real elastic = bounceHeight(1.0f, ccd);
        const Real inelastic = bounceHeight(0.0f, ccd);
        std::printf("restitution[ccd %s]: e=1 -> %.3f, e=0 -> %.3f\n", ccd ? "on " : "off", elastic, inelastic);
        TST_REQUIRE(elastic > 2.4f);     // rebounds to >75% of the 2.5 m drop (center 3.0 -> rest 0.5)
        TST_REQUIRE(inelastic < 0.75f);  // settles at the resting height, no bounce
    }
}

// Coulomb friction on a flat slab given an initial horizontal velocity: frictionless keeps the
// speed; high friction damps it out. (A low-profile slab to limit tipping.)
TST_CASE(physics, integration, friction) {
    auto run = [](Real friction) {
        auto w = makeWorld(Vec3(0, -9.81f, 0), 12, 2);
        addGroundPlane(*w, friction);
        BodyDef b;
        b.type = BodyType::Dynamic; b.mass = 1.0f;
        b.collider.type = ColliderDesc::Type::Box;
        b.collider.box = Box{ Vec3(0.5f, 0.15f, 0.5f) };
        b.material.friction = friction;
        b.material.restitution = 0.0f;
        b.position = Vec3(0, 0.15f, 0);
        b.linearVelocity = Vec3(3, 0, 0);
        const BodyHandle h = w->createBody(b);
        for (int i = 0; i < 90; ++i) w->step(1.0f / 120.0f);   // 0.75 s
        return w->linearVelocities()[h.index].x;
    };
    const Real slick = run(0.0f);
    const Real grippy = run(1.0f);
    std::printf("friction: vx frictionless=%.3f, high=%.3f\n", slick, grippy);
    TST_REQUIRE(slick > 2.5f);    // nothing slows it
    TST_REQUIRE(grippy < 1.0f);   // friction (mu=1, decel ~g) brings it near a stop
}

// Head-on elastic collision of equal spheres (no gravity, frictionless, e=1): the moving body
// stops and the struck body carries the velocity away (1-D elastic exchange).
TST_CASE(physics, integration, elastic_swap) {
    auto w = makeWorld(Vec3(0), 16, 2);
    auto mkSphere = [&](Vec3 pos, Vec3 vel) {
        BodyDef d;
        d.type = BodyType::Dynamic; d.mass = 1.0f;
        d.collider.type = ColliderDesc::Type::Sphere;
        d.collider.sphere = Sphere{ 0.5f };
        d.material.restitution = 1.0f; d.material.friction = 0.0f;
        d.position = pos; d.linearVelocity = vel;
        return w->createBody(d);
    };
    const BodyHandle a = mkSphere(Vec3(-2, 0, 0), Vec3(4, 0, 0));
    const BodyHandle b = mkSphere(Vec3(0, 0, 0), Vec3(0));
    for (int i = 0; i < 150; ++i) w->step(1.0f / 120.0f);
    const Real va = w->linearVelocities()[a.index].x;
    const Real vb = w->linearVelocities()[b.index].x;
    std::printf("elastic swap: A.vx=%.3f B.vx=%.3f (expect ~0 and ~4)\n", va, vb);
    TST_REQUIRE(vb > 3.2f);          // struck body carries the momentum
    TST_REQUIRE(std::fabs(va) < 0.8f); // incoming body nearly stops
}

// A three-box stack must stay standing and near its column (stresses the multi-point solver
// more than a two-box stack).
TST_CASE(physics, integration, tall_stack) {
    auto w = makeWorld(Vec3(0, -9.81f, 0), 20, 2);
    addGroundPlane(*w, 0.9f);
    BodyHandle bh[3];
    for (int k = 0; k < 3; ++k) {
        BodyDef d;
        d.type = BodyType::Dynamic; d.mass = 1.0f;
        d.collider.type = ColliderDesc::Type::Box;
        d.collider.box = Box{ Vec3(0.5f) };
        d.material.friction = 0.9f; d.material.restitution = 0.0f;
        d.position = Vec3(0, 0.5f + k, 0);
        bh[k] = w->createBody(d);
    }
    for (int i = 0; i < 500; ++i) w->step(1.0f / 120.0f);
    for (int k = 0; k < 3; ++k) {
        const auto p = w->pose(bh[k]);
        std::printf("tall_stack[%d]: y=%.3f x=%.3f z=%.3f\n", k, p.position.y, p.position.x, p.position.z);
        TST_REQUIRE(std::fabs(p.position.y - (0.5f + k)) < 0.15f);
        TST_REQUIRE(std::fabs(p.position.x) < 0.2f && std::fabs(p.position.z) < 0.2f);
    }
}

// The two broadphases must feed the same narrowphase and converge to the same rest state.
TST_CASE(physics, integration, broadphase_equivalence) {
    auto build = [](BroadphaseKind bp) {
        auto w = makeWorld(Vec3(0, -9.81f, 0), 8, 1, bp);
        addGroundPlane(*w, 0.8f);
        for (int x = 0; x < 4; ++x)
            for (int z = 0; z < 4; ++z) {
                BodyDef s;
                s.type = BodyType::Dynamic; s.mass = 1.0f;
                s.collider.type = ColliderDesc::Type::Sphere;
                s.collider.sphere = Sphere{ 0.5f };
                s.material.friction = 0.8f;
                s.position = Vec3(x * 0.7f, 1.0f, z * 0.7f);   // overlapping cluster
                w->createBody(s);
            }
        return w;
    };
    auto sap = build(BroadphaseKind::SweepAndPrune);
    auto grid = build(BroadphaseKind::UniformGrid);
    for (int i = 0; i < 200; ++i) { sap->step(1.0f / 120.0f); grid->step(1.0f / 120.0f); }
    const auto ps = sap->poses();
    const auto pg = grid->poses();
    TST_REQUIRE(ps.size() == pg.size());
    Real maxErr = 0;
    for (size_t k = 0; k < ps.size(); ++k) maxErr = std::max(maxErr, glm::length(ps[k].position - pg[k].position));
    std::printf("broadphase_equivalence: max pos err SAP vs grid = %.3e\n", maxErr);
    TST_REQUIRE(maxErr < 1e-3f);
}

// A kinematic body advances by its prescribed velocity, ignores gravity, and is not pushed back
// by the solver (infinite mass).
TST_CASE(physics, integration, kinematic_motion) {
    auto w = makeWorld(Vec3(0, -9.81f, 0), 8, 1);
    BodyDef k;
    k.type = BodyType::Kinematic;
    k.collider.type = ColliderDesc::Type::Box;
    k.collider.box = Box{ Vec3(0.5f) };
    k.position = Vec3(0, 5, 0);
    k.linearVelocity = Vec3(1, 0, 0);
    const BodyHandle h = w->createBody(k);
    for (int i = 0; i < 120; ++i) w->step(1.0f / 120.0f);   // 1.0 s
    const auto p = w->pose(h);
    std::printf("kinematic_motion: pos=(%.3f, %.3f, %.3f) (expect ~1, 5, 0)\n", p.position.x, p.position.y, p.position.z);
    TST_REQUIRE(std::fabs(p.position.x - 1.0f) < 1e-3f);   // moved by v·t
    TST_REQUIRE(std::fabs(p.position.y - 5.0f) < 1e-4f);   // gravity does NOT pull a kinematic body down
}

// A kinematic body drives dynamics but is not pushed back: an upward-moving kinematic platform
// carries a resting dynamic box with it, and the platform itself stays on its scripted path.
TST_CASE(physics, integration, kinematic_pushes_dynamic) {
    auto w = makeWorld(Vec3(0, -9.81f, 0), 12, 2);
    BodyDef plat;
    plat.type = BodyType::Kinematic;
    plat.collider.type = ColliderDesc::Type::Box;
    plat.collider.box = Box{ Vec3(2.0f, 0.25f, 2.0f) };
    plat.position = Vec3(0, 0.25f, 0);
    plat.linearVelocity = Vec3(0, 0.5f, 0);            // rising at 0.5 m/s
    const BodyHandle ph = w->createBody(plat);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box;
    box.collider.box = Box{ Vec3(0.5f) };
    box.material.friction = 0.8f; box.material.restitution = 0.0f;
    box.position = Vec3(0, 1.0f, 0);                   // resting on the platform top (0.5)
    const BodyHandle bh = w->createBody(box);

    for (int i = 0; i < 120; ++i) w->step(1.0f / 120.0f);   // 1.0 s -> platform rises 0.5 m
    const auto pp = w->pose(ph);
    const auto pb = w->pose(bh);
    std::printf("kinematic push: platform.y=%.3f box.y=%.3f (both +~0.5)\n", pp.position.y, pb.position.y);
    TST_REQUIRE(std::fabs(pp.position.y - 0.75f) < 1e-3f);  // platform followed its path (0.25 -> 0.75)
    TST_REQUIRE(pb.position.y > 1.4f && pb.position.y < 1.6f);  // box carried up ~0.5, still resting on top
}
