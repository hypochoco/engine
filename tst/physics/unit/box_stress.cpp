//
//  box_stress.cpp
//  engine::tst / physics / unit
//
//  Reproduces the sim-2 "freeze / tunnel near a rock" bug: a small dynamic box (the player) meeting a
//  large, FLATTENED, ORIENTED box (a rock collider). Exercises the box-vs-box narrowphase (which runs
//  GJK/EPA under the hood) directly and through a stepped realtime world, checking that (a) it returns
//  a blocking contact, (b) the moving box does not tunnel through, and (c) it does so promptly.
//

#include <chrono>
#include <cmath>
#include <cstdio>

#include <glm/gtc/quaternion.hpp>

#include "engine/physics/collision/box_box.h"
#include "engine/physics/collision/convex.h"
#include "engine/physics/collision/support.h"
#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;
using clk = std::chrono::steady_clock;

// A single overlapping box-vs-box query must return a blocking manifold quickly (not spin in EPA).
TST_CASE(physics, unit, box_box_flat_overlap) {
    const Box player{ Vec3(0.5f, 0.5f, 0.5f) };
    const Box rock{ Vec3(1.0f, 0.45f, 1.0f) };                 // flattened, like a rock collider
    const Quat rockRot = glm::angleAxis(0.6f, Vec3(0, 1, 0));  // yaw
    const Vec3 rockC(0, 0.45f, 0);

    const Vec3 pC(1.2f, 0.5f, 0.0f);                           // overlapping the rock's +x side
    Contact cs[4];
    const auto t0 = clk::now();
    const int n = collide::boxVsBox(pC, Quat(1, 0, 0, 0), player, rockC, rockRot, rock, cs);
    const double ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
    std::printf("box_box_flat_overlap: n=%d in %.3f ms, normal=(%.2f,%.2f,%.2f) sep=%.3f\n",
                n, ms, cs[0].normal.x, cs[0].normal.y, cs[0].normal.z, cs[0].separation);
    TST_REQUIRE_MSG(ms < 200.0, "single box-vs-box query is pathologically slow (EPA blowup)");
    TST_REQUIRE_MSG(n > 0, "overlapping boxes must produce a contact");
    TST_REQUIRE_MSG(cs[0].separation < 0.0f, "overlap ⇒ negative separation");
}

// Sweep the player box straight through the rock; every query must terminate quickly, and the deep
// overlaps must report contacts (no silent miss ⇒ no tunneling).
TST_CASE(physics, unit, box_box_sweep) {
    const Box player{ Vec3(0.5f) };
    const Box rock{ Vec3(1.0f, 0.45f, 1.0f) };
    const Quat rockRot = glm::angleAxis(0.6f, Vec3(0, 1, 0));
    const Vec3 rockC(0, 0.45f, 0);

    double worst = 0; int contacts = 0, overlaps = 0;
    for (int i = 0; i <= 60; ++i) {
        const float x = 2.0f - 4.0f * (i / 60.0f);       // +2 → -2 through the rock
        Contact cs[4];
        const auto t0 = clk::now();
        const int n = collide::boxVsBox(Vec3(x, 0.5f, 0), Quat(1, 0, 0, 0), player, rockC, rockRot, rock, cs);
        worst = std::max(worst, std::chrono::duration<double, std::milli>(clk::now() - t0).count());
        if (n > 0) ++contacts;
        if (std::fabs(x) < 1.2f) ++overlaps;             // clearly-overlapping samples
    }
    std::printf("box_box_sweep: contacts=%d overlaps≈%d worst=%.3f ms\n", contacts, overlaps, worst);
    TST_REQUIRE_MSG(worst < 200.0, "a box-vs-box query in the sweep is pathologically slow");
    TST_REQUIRE_MSG(contacts >= overlaps, "overlapping samples must all report contacts (else tunneling)");
}

// End-to-end: a dynamic player box driven into a static rock box through the realtime world must be
// BLOCKED (not tunnel to the far side) and the whole run must complete promptly.
TST_CASE(physics, integration, box_into_rock_blocks) {
    WorldDef wd;
    wd.gravity = Vec3(0, -18.0f, 0);
    wd.substeps = 4;
    auto w = createPhysicsWorld(Backend::Realtime, wd);

    BodyDef rock;
    rock.type = BodyType::Static;
    rock.collider.type = ColliderDesc::Type::Box;
    rock.collider.box = Box{ Vec3(1.0f, 0.45f, 1.0f) };
    rock.orientation = glm::angleAxis(0.6f, Vec3(0, 1, 0));
    rock.position = Vec3(0, 0.45f, 0);
    rock.material.friction = 0.7f;
    w->createBody(rock);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box;
    box.collider.box = Box{ Vec3(0.5f) };
    box.material.friction = 0.6f;
    box.position = Vec3(2.2f, 0.5f, 0.0f);
    box.linearVelocity = Vec3(-5.0f, 0.0f, 0.0f);   // launched toward the rock once
    const BodyHandle bh = w->createBody(box);

    const auto t0 = clk::now();
    for (int s = 0; s < 240; ++s) w->step(1.0f / 60.0f);   // free flight — collision must stop it
    const double ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
    const float endX = w->pose(bh).position.x;
    std::printf("box_into_rock_blocks: endX=%.2f in %.1f ms (rock spans |x|≲1.4)\n", endX, ms);
    TST_REQUIRE_MSG(ms < 2000.0, "240 steps took too long — collision is pathologically slow");
    TST_REQUIRE_MSG(endX > -0.6f, "player tunneled through the rock (should be blocked on the +x side)");
}

// Mimics the sim-2 character controller: each step read the body velocity, ACCUMULATE horizontal
// acceleration toward the wish direction (clamped to a top speed), preserve vertical, and write it
// back with setBodyState — then step. The solver's contact must still keep the box out of the rock.
TST_CASE(physics, integration, controller_into_rock_blocks) {
    WorldDef wd;
    wd.gravity = Vec3(0, -18.0f, 0);
    wd.substeps = 4;
    auto w = createPhysicsWorld(Backend::Realtime, wd);

    BodyDef ground;
    ground.type = BodyType::Static;
    ground.collider.type = ColliderDesc::Type::Plane;
    ground.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
    ground.material.friction = 0.8f;
    w->createBody(ground);

    BodyDef rock;
    rock.type = BodyType::Static;
    rock.collider.type = ColliderDesc::Type::Box;
    rock.collider.box = Box{ Vec3(1.0f, 0.45f, 1.0f) };
    rock.orientation = glm::angleAxis(0.6f, Vec3(0, 1, 0));
    rock.position = Vec3(0, 0.45f, 0);
    rock.material.friction = 0.7f;
    w->createBody(rock);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box;
    box.collider.box = Box{ Vec3(0.5f) };
    box.material.friction = 0.6f;
    box.position = Vec3(3.0f, 0.5f, 0.0f);
    const BodyHandle bh = w->createBody(box);

    constexpr float dt = 1.0f / 60.0f, accel = 45.0f, moveSpeed = 6.0f;
    float minX = 3.0f;
    for (int s = 0; s < 300; ++s) {
        const auto pose = w->pose(bh);
        Vec3 v = w->linearVelocities()[bh.index];
        Vec3 horiz(v.x, 0.0f, v.z);
        horiz += Vec3(-1, 0, 0) * accel * dt;                 // wish = -x (into the rock)
        const float sp = glm::length(horiz);
        if (sp > moveSpeed) horiz *= moveSpeed / sp;
        w->setBodyState(bh, pose.position, pose.rotation, Vec3(horiz.x, v.y, horiz.z),
                        w->angularVelocities()[bh.index]);
        w->step(dt);
        minX = std::min(minX, w->pose(bh).position.x);
    }
    const float endX = w->pose(bh).position.x;
    std::printf("controller_into_rock_blocks: endX=%.2f minX=%.2f (rock spans |x|≲1.4)\n", endX, minX);
    TST_REQUIRE_MSG(minX > -0.8f, "controller pushed the box through the rock (tunneling)");
}
