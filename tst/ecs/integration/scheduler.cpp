//
//  scheduler_test.cpp
//  engine::tst
//
//  Driver test for the ECS ordered scheduler + resources: a gravity system then an integrate
//  system, both reading a Time{dt} resource and iterating via queries. Runs two fixed steps
//  and checks the result against a hand computation (deterministic).
//

#include "harness/harness.h"
#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>

#include "engine/core/core.h"     // engine::Transform
#include "engine/ecs/ecs.h"

namespace {
struct Velocity { glm::vec3 v{0.0f}; };
}   // uses the shared engine::Time resource (core/time.h) for dt

TST_CASE(ecs, integration, scheduler) {
    using namespace engine;
    using namespace engine::ecs;

    World world;
    Entity e = world.spawn(Transform{}, Velocity{ glm::vec3(0.0f) });
    for (int i = 0; i < 9; ++i) world.spawn(Transform{}, Velocity{ glm::vec3(0.0f) });

    world.setResource(Time{ 0.5f });
    assert(world.getResource<Time>()->dt == 0.5f);

    Schedule schedule;
    schedule.add("gravity", [](World& w) {
        const float dt = w.getResource<Time>()->dt;
        w.query<Velocity>().each([&](Entity, Velocity& vel) { vel.v.y -= 10.0f * dt; });
    });
    schedule.add("integrate", [](World& w) {
        const float dt = w.getResource<Time>()->dt;
        w.query<Transform, Velocity>().each([&](Entity, Transform& t, Velocity& vel) {
            t.position += vel.v * dt;
        });
    });
    assert(schedule.size() == 2);

    // Two steps at dt = 0.5, gravity = -10:
    //  step 1: vy = -5 ; pos.y += -5*0.5 = -2.5
    //  step 2: vy = -10; pos.y += -10*0.5 = -5   -> pos.y = -7.5
    schedule.run(world);
    schedule.run(world);

    const Transform* t = world.get<Transform>(e);
    const Velocity*  v = world.get<Velocity>(e);
    std::printf("after 2 steps: pos.y = %.3f, vel.y = %.3f\n", t->position.y, v->v.y);

    assert(std::fabs(v->v.y - (-10.0f)) < 1e-4f);
    assert(std::fabs(t->position.y - (-7.5f)) < 1e-4f);

    // All 10 entities advanced identically (deterministic).
    int moved = 0;
    world.query<Transform>().each([&](Entity, Transform& tr) {
        if (std::fabs(tr.position.y - (-7.5f)) < 1e-4f) ++moved;
    });
    assert(moved == 10);

    std::printf("scheduler ok\n");
}
