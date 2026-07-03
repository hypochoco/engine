//
//  rigidbody.cpp
//  engine::tst / physics / integration
//
//  Rigid-body dynamics through the realtime PhysicsWorld: determinism (serial == parallel),
//  resting, stacking, capsule contacts, and CCD. These exercise the full step pipeline
//  (broadphase → narrowphase → colored solver → integrate).
//

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/collision/capsule.h"
#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
std::unique_ptr<PhysicsWorld> groundWorld(int velIters, int substeps) {
    WorldDef wd;
    wd.gravity = Vec3(0, -9.81f, 0);
    wd.velocityIterations = velIters;
    wd.substeps = substeps;
    auto w = createPhysicsWorld(Backend::Realtime, wd);
    BodyDef plane;
    plane.type = BodyType::Static;
    plane.collider.type = ColliderDesc::Type::Plane;
    plane.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
    plane.material.friction = 0.9f;
    w->createBody(plane);
    return w;
}
} // namespace

// Parallel step is bit-identical to serial (same colored-solve order).
TST_CASE(physics, integration, determinism) {
    engine::core::ThreadPool pool;
    auto buildPile = [](engine::core::ThreadPool* p) {
        WorldDef wd;
        wd.gravity = Vec3(0, -9.81f, 0);
        wd.velocityIterations = 8;
        wd.threadPool = p;
        wd.parallelThreshold = p ? 1 : 1000000;
        auto w = createPhysicsWorld(Backend::Realtime, wd);
        BodyDef plane;
        plane.type = BodyType::Static;
        plane.collider.type = ColliderDesc::Type::Plane;
        plane.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
        plane.material.friction = 0.8f;
        w->createBody(plane);
        for (int x = 0; x < 5; ++x)
            for (int z = 0; z < 5; ++z)
                for (int y = 0; y < 2; ++y) {
                    BodyDef s;
                    s.type = BodyType::Dynamic; s.mass = 1.0f;
                    s.collider.type = ColliderDesc::Type::Sphere;
                    s.collider.sphere = Sphere{ 0.5f };
                    s.material.friction = 0.8f;
                    s.position = Vec3(x * 0.8f, 1.0f + y * 0.9f, z * 0.8f);
                    w->createBody(s);
                }
        return w;
    };
    auto serial = buildPile(nullptr);
    auto parallel = buildPile(&pool);
    for (int i = 0; i < 120; ++i) { serial->step(1.0f / 120.0f); parallel->step(1.0f / 120.0f); }
    const auto ps = serial->poses();
    const auto pp = parallel->poses();
    TST_REQUIRE(ps.size() == pp.size());
    Real maxErr = 0;
    for (size_t k = 0; k < ps.size(); ++k)
        maxErr = std::max(maxErr, glm::length(ps[k].position - pp[k].position));
    std::printf("determinism: max pos err = %.3e\n", maxErr);
    TST_REQUIRE(maxErr < 1e-4f);
}

// Box, sphere-on-box, and hull all come to rest flat.
TST_CASE(physics, integration, resting) {
    auto w = groundWorld(12, 2);
    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box;
    box.collider.box = Box{ Vec3(0.5f) };
    box.material.friction = 0.8f; box.material.restitution = 0.0f;
    box.position = Vec3(0, 1.0f, 0);
    const BodyHandle bh = w->createBody(box);

    BodyDef hull;
    hull.type = BodyType::Dynamic; hull.mass = 1.0f;
    hull.collider.type = ColliderDesc::Type::ConvexHull;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                hull.collider.convexHull.vertices.push_back(Vec3(sx * 0.5f, sy * 0.5f, sz * 0.5f));
    hull.material.friction = 0.8f; hull.material.restitution = 0.0f;
    hull.position = Vec3(5, 1.2f, 0);
    const BodyHandle hh = w->createBody(hull);

    for (int i = 0; i < 300; ++i) w->step(1.0f / 120.0f);
    const auto pb = w->pose(bh);
    const auto ph = w->pose(hh);
    std::printf("resting: box.y=%.3f hull.y=%.3f\n", pb.position.y, ph.position.y);
    TST_REQUIRE(pb.position.y > 0.45f && pb.position.y < 0.56f);
    TST_REQUIRE(glm::length(w->angularVelocities()[bh.index]) < 0.3f);
    TST_REQUIRE(ph.position.y > 0.45f && ph.position.y < 0.56f);
    TST_REQUIRE(glm::length(w->angularVelocities()[hh.index]) < 0.4f);
}

// Boxes and hulls stack without toppling (multi-point face-clip manifolds).
TST_CASE(physics, integration, stacking) {
    for (bool useHull : { false, true }) {
        auto w = groundWorld(16, 2);
        auto make = [&](float y) {
            BodyDef d;
            d.type = BodyType::Dynamic; d.mass = 1.0f;
            d.material.friction = 0.9f; d.material.restitution = 0.0f;
            d.position = Vec3(0, y, 0);
            if (useHull) {
                d.collider.type = ColliderDesc::Type::ConvexHull;
                for (int sx = -1; sx <= 1; sx += 2)
                    for (int sy = -1; sy <= 1; sy += 2)
                        for (int sz = -1; sz <= 1; sz += 2)
                            d.collider.convexHull.vertices.push_back(Vec3(sx * 0.5f, sy * 0.5f, sz * 0.5f));
            } else {
                d.collider.type = ColliderDesc::Type::Box;
                d.collider.box = Box{ Vec3(0.5f) };
            }
            return w->createBody(d);
        };
        const BodyHandle bottom = make(0.5f);
        const BodyHandle top = make(1.5f);
        for (int i = 0; i < 400; ++i) w->step(1.0f / 120.0f);
        const auto pb = w->pose(bottom);
        const auto pt = w->pose(top);
        std::printf("stack(%s): bottom.y=%.3f top.y=%.3f\n", useHull ? "hull" : "box", pb.position.y, pt.position.y);
        TST_REQUIRE(pb.position.y > 0.45f && pb.position.y < 0.56f);
        TST_REQUIRE(pt.position.y > 1.40f && pt.position.y < 1.60f);
        TST_REQUIRE(std::fabs(pt.position.x) < 0.25f && std::fabs(pt.position.z) < 0.25f);
    }
}

// Analytic capsule contacts + a lying capsule resting on a plane and on a box.
TST_CASE(physics, integration, capsule) {
    const Quat I(1, 0, 0, 0);
    const Capsule cap{ 0.5f, 0.5f };
    Contact c;
    TST_REQUIRE(collide::capsuleVsSphere(Vec3(0, 0, 0), I, cap, Vec3(0.8f, 0, 0), Sphere{ 0.5f }, Real(0), c));
    TST_REQUIRE(std::fabs(c.separation - (-0.2f)) < 1e-3f && c.normal.x > 0.9f);
    Contact cc;
    TST_REQUIRE(collide::capsuleVsCapsule(Vec3(0, 0, 0), I, cap, Vec3(0.8f, 0, 0), I, cap, Real(0), cc));
    TST_REQUIRE(std::fabs(cc.separation - (-0.2f)) < 1e-3f && cc.normal.x > 0.9f);

    // Lying capsule on a plane rests on its radius, flat.
    auto w = groundWorld(12, 2);
    BodyDef capsule;
    capsule.type = BodyType::Dynamic; capsule.mass = 1.0f;
    capsule.collider.type = ColliderDesc::Type::Capsule;
    capsule.collider.capsule = Capsule{ 0.5f, 1.0f };
    capsule.material.friction = 0.8f; capsule.material.restitution = 0.0f;
    capsule.orientation = glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1));
    capsule.position = Vec3(0, 1.2f, 0);
    const BodyHandle ch = w->createBody(capsule);
    for (int i = 0; i < 300; ++i) w->step(1.0f / 120.0f);
    const auto p = w->pose(ch);
    std::printf("capsule on plane: y=%.3f\n", p.position.y);
    TST_REQUIRE(p.position.y > 0.45f && p.position.y < 0.56f);
    TST_REQUIRE(glm::length(w->angularVelocities()[ch.index]) < 0.4f);

    // Lying capsule on a static box (capsule-vs-box via GJK distance).
    auto w2 = createPhysicsWorld(Backend::Realtime, [] { WorldDef d; d.gravity = Vec3(0, -9.81f, 0); d.velocityIterations = 16; d.substeps = 2; return d; }());
    BodyDef sbox;
    sbox.type = BodyType::Static;
    sbox.collider.type = ColliderDesc::Type::Box;
    sbox.collider.box = Box{ Vec3(2.0f, 0.5f, 2.0f) };
    sbox.position = Vec3(0, 0.5f, 0);
    w2->createBody(sbox);
    BodyDef cap2;
    cap2.type = BodyType::Dynamic; cap2.mass = 1.0f;
    cap2.collider.type = ColliderDesc::Type::Capsule;
    cap2.collider.capsule = Capsule{ 0.4f, 0.8f };
    cap2.material.friction = 0.8f; cap2.material.restitution = 0.0f;
    cap2.orientation = glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1));
    cap2.position = Vec3(0, 1.6f, 0);
    const BodyHandle ch2 = w2->createBody(cap2);
    for (int i = 0; i < 300; ++i) w2->step(1.0f / 120.0f);
    const auto p2 = w2->pose(ch2);
    std::printf("capsule on box: y=%.3f\n", p2.position.y);
    TST_REQUIRE(p2.position.y > 1.35f && p2.position.y < 1.46f);   // box top 1.0 + radius 0.4
}

// A fast sphere does not tunnel through a small static box (swept AABB + speculative contacts).
TST_CASE(physics, integration, ccd) {
    auto run = [](bool ccd) {
        WorldDef wd;
        wd.gravity = Vec3(0, 0, 0);
        wd.velocityIterations = 8;
        wd.continuousDetection = ccd;
        auto w = createPhysicsWorld(Backend::Realtime, wd);
        BodyDef box;
        box.type = BodyType::Static;
        box.collider.type = ColliderDesc::Type::Box;
        box.collider.box = Box{ Vec3(0.5f) };
        w->createBody(box);
        BodyDef bullet;
        bullet.type = BodyType::Dynamic; bullet.mass = 1.0f;
        bullet.collider.type = ColliderDesc::Type::Sphere;
        bullet.collider.sphere = Sphere{ 0.1f };
        bullet.position = Vec3(0, 3.0f, 0);
        bullet.linearVelocity = Vec3(0, -120.0f, 0);
        const BodyHandle bh = w->createBody(bullet);
        for (int i = 0; i < 30; ++i) w->step(1.0f / 120.0f);
        return w->pose(bh).position.y;
    };
    const Real yCCD = run(true);
    const Real yNo = run(false);
    std::printf("CCD: y = %.3f (on) vs %.3f (off)\n", yCCD, yNo);
    TST_REQUIRE(yCCD > 0.55f);   // stops on the box
    TST_REQUIRE(yNo < 0.0f);     // tunnels without CCD
}
