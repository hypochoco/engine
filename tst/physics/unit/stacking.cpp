//
//  stacking.cpp
//  engine::tst / physics / integration
//
//  Resting stability: a dynamic box settling on top of another body must STAY put (not creep, jitter
//  off, or get ejected). Two cases isolate the cause: a FLAT top (box-on-box) tests the contact solver
//  itself; a ROUNDED top (box-on-hull) tests whether the rock's curved surface is the destabilizer.
//

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
std::unique_ptr<PhysicsWorld> makeWorld() {
    WorldDef wd;
    wd.gravity = Vec3(0, -18.0f, 0);
    wd.substeps = 4;
    wd.velocityIterations = 8;
    if (const char* s = std::getenv("PHYS_BAUM")) wd.solver.contactBaumgarte = std::atof(s);
    if (const char* s = std::getenv("PHYS_SLOP")) wd.solver.contactSlop = std::atof(s);
    if (const char* s = std::getenv("PHYS_SUBSTEPS")) wd.substeps = std::atoi(s);
    auto w = createPhysicsWorld(Backend::Realtime, wd);
    BodyDef ground;
    ground.type = BodyType::Static;
    ground.collider.type = ColliderDesc::Type::Plane;
    ground.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
    ground.material.friction = 0.9f;
    w->createBody(ground);
    return w;
}
} // namespace

// A box settling on a static box (flat contact) must stay put — this exercises the contact solver.
TST_CASE(physics, integration, rest_box_on_box) {
    auto w = makeWorld();
    BodyDef ped;                             // static pedestal, top face at y = 1.0
    ped.type = BodyType::Static;
    ped.collider.type = ColliderDesc::Type::Box;
    ped.collider.box = Box{ Vec3(1.0f, 0.5f, 1.0f) };
    ped.position = Vec3(0, 0.5f, 0);
    ped.material.friction = 0.9f;
    w->createBody(ped);

    BodyDef top;
    top.type = BodyType::Dynamic; top.mass = 1.0f;
    top.collider.type = ColliderDesc::Type::Box;
    top.collider.box = Box{ Vec3(0.5f) };
    top.material.friction = 0.9f;
    top.position = Vec3(0, 1.55f, 0);        // just above the pedestal top
    const BodyHandle bh = w->createBody(top);

    for (int s = 0; s < 120; ++s) w->step(1.0f / 60.0f);   // settle
    const Vec3 settled = w->pose(bh).position;
    for (int s = 0; s < 300; ++s) w->step(1.0f / 60.0f);   // hold
    const Vec3 end = w->pose(bh).position;

    const float drift = glm::length(glm::vec2(end.x - settled.x, end.z - settled.z));
    std::printf("rest_box_on_box: settled=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) drift=%.3f\n",
                settled.x, settled.y, settled.z, end.x, end.y, end.z, drift);
    TST_REQUIRE_MSG(end.y > 1.4f, "top box fell off the pedestal");
    TST_REQUIRE_MSG(drift < 0.05f, "top box drifted/crept — resting instability");
}

// A box settling on a rounded (flattened icosphere-ish) hull — the rock case (diagnostic).
TST_CASE(physics, integration, rest_box_on_hull) {
    auto w = makeWorld();

    std::vector<Vec3> verts;
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    const Vec3 ico[12] = { {-1,t,0},{1,t,0},{-1,-t,0},{1,-t,0},{0,-1,t},{0,1,t},
                           {0,-1,-t},{0,1,-t},{t,0,-1},{t,0,1},{-t,0,-1},{-t,0,1} };
    for (const Vec3& v : ico) verts.push_back(glm::normalize(v) * Vec3(1.0f, 0.45f, 1.0f));

    BodyDef rock;
    rock.type = BodyType::Static;
    rock.collider.type = ColliderDesc::Type::ConvexHull;
    rock.collider.convexHull.vertices = verts;
    rock.position = Vec3(0, 0.45f, 0);
    rock.material.friction = 0.9f;
    w->createBody(rock);

    BodyDef top;
    top.type = BodyType::Dynamic; top.mass = 1.0f;
    top.collider.type = ColliderDesc::Type::Box;
    top.collider.box = Box{ Vec3(0.5f) };
    top.material.friction = 0.9f;
    top.position = Vec3(0, 1.5f, 0);
    const BodyHandle bh = w->createBody(top);

    for (int s = 0; s < 120; ++s) w->step(1.0f / 60.0f);
    const Vec3 settled = w->pose(bh).position;
    for (int s = 0; s < 300; ++s) w->step(1.0f / 60.0f);
    const Vec3 end = w->pose(bh).position;
    const float drift = glm::length(glm::vec2(end.x - settled.x, end.z - settled.z));
    std::printf("rest_box_on_hull: settled=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) drift=%.3f\n",
                settled.x, settled.y, settled.z, end.x, end.y, end.z, drift);
    // Not asserted (rounded top may legitimately shed the box) — diagnostic print.
}

// A WIDE FLAT-TOPPED hull (drum: a flat top disk wider than the cube) — the box must rest stably.
// Validates the fix: a flat top at least as wide as the resting box gives a stable face contact.
TST_CASE(physics, integration, rest_box_on_flattop_hull) {
    auto w = makeWorld();

    std::vector<Vec3> verts;                           // drum: top disk r=0.9 @ y=0.3, base r=1.0 @ y=-0.2
    for (int k = 0; k < 10; ++k) {
        const float a = 6.2831853f * k / 10.0f;
        verts.push_back(Vec3(std::cos(a) * 0.9f, 0.30f, std::sin(a) * 0.9f));
        verts.push_back(Vec3(std::cos(a) * 1.0f, -0.20f, std::sin(a) * 1.0f));
    }

    BodyDef rock;
    rock.type = BodyType::Static;
    rock.collider.type = ColliderDesc::Type::ConvexHull;
    rock.collider.convexHull.vertices = verts;
    rock.position = Vec3(0, 0.45f, 0);                 // top face at y = 0.75
    rock.material.friction = 0.9f;
    w->createBody(rock);

    BodyDef top;
    top.type = BodyType::Dynamic; top.mass = 1.0f;
    top.collider.type = ColliderDesc::Type::Box;
    top.collider.box = Box{ Vec3(0.5f) };
    top.material.friction = 0.9f;
    top.position = Vec3(0, 1.30f, 0);                  // gently above the flat top (rest ≈ 1.25)
    const BodyHandle bh = w->createBody(top);

    for (int s = 0; s < 120; ++s) w->step(1.0f / 60.0f);
    const Vec3 settled = w->pose(bh).position;
    for (int s = 0; s < 300; ++s) w->step(1.0f / 60.0f);
    const Vec3 end = w->pose(bh).position;
    const float drift = glm::length(glm::vec2(end.x - settled.x, end.z - settled.z));
    std::printf("rest_box_on_flattop_hull: settled=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) drift=%.3f\n",
                settled.x, settled.y, settled.z, end.x, end.y, end.z, drift);
    TST_REQUIRE_MSG(end.y > 1.1f, "box fell off the wide flat-topped hull");
    TST_REQUIRE_MSG(drift < 0.05f, "box drifted on the flat-topped hull — manifold instability");
}

// Edge/corner impulse: a box overhanging the rim of a flat-top drum must tip off GENTLY, not be
// launched. Reproduces the sim "getting shoved off the boulder edge with too much force" report.
TST_CASE(physics, integration, edge_no_launch) {
    auto w = makeWorld();

    std::vector<Vec3> verts;                            // drum: top r=1.15 @ y=1.2, base r=1.4 @ y=0
    const int seg = 11;
    for (int k = 0; k < seg; ++k) {
        const float a = 6.2831853f * k / seg;
        verts.push_back(Vec3(std::cos(a) * 1.15f, 1.2f, std::sin(a) * 1.15f));
        verts.push_back(Vec3(std::cos(a) * 1.40f, 0.0f, std::sin(a) * 1.40f));
    }
    BodyDef rock;
    rock.type = BodyType::Static;
    rock.collider.type = ColliderDesc::Type::ConvexHull;
    rock.collider.convexHull.vertices = verts;
    rock.material.friction = 0.8f;
    w->createBody(rock);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box;
    box.collider.box = Box{ Vec3(0.5f) };
    box.material.friction = 0.6f;
    box.position = Vec3(1.10f, 1.75f, 0.0f);           // overhanging the top rim, resting height
    const BodyHandle bh = w->createBody(box);

    float maxSpeed = 0.0f;
    for (int s = 0; s < 180; ++s) {
        w->step(1.0f / 60.0f);
        maxSpeed = std::max(maxSpeed, glm::length(w->linearVelocities()[bh.index]));
    }
    const Vec3 end = w->pose(bh).position;
    std::printf("edge_no_launch: maxSpeed=%.2f end=(%.2f,%.2f,%.2f)\n", maxSpeed, end.x, end.y, end.z);
    // A box settling/tipping under gravity from ~0.05m should never exceed a few m/s; a launch spike is
    // several× that. This asserts no propulsion (it may still slide off, just not be flung).
    TST_REQUIRE_MSG(maxSpeed < 5.0f, "box was launched off the edge (contact impulse spike)");
}
