//
//  ecs_test.cpp
//  engine::tst
//
//  Driver test for engine::ecs: spawn entities across archetypes, query with .each/.chunks,
//  mutate through queries, destroy + verify swap-remove and stale-handle invalidation.
//  Headless, no graphics.
//

#include "harness/harness.h"
#include <cstdio>

#include <glm/glm.hpp>

#include "engine/core/core.h"        // engine::Transform
#include "engine/ecs/ecs.h"

namespace { struct Velocity { glm::vec3 v{0.0f}; }; }

TST_CASE(ecs, unit, entities) {
    using namespace engine;
    using namespace engine::ecs;

    World world;

    // Archetype A: {Transform, Velocity} x1000 ; Archetype B: {Transform} x500.
    for (int i = 0; i < 1000; ++i)
        world.spawn(Transform{ .position = glm::vec3(float(i), 0, 0) }, Velocity{ glm::vec3(1, 2, 3) });
    for (int i = 0; i < 500; ++i)
        world.spawn(Transform{ .position = glm::vec3(0, float(i), 0) });

    assert(world.size() == 1500);

    // query<Transform> matches BOTH archetypes -> 1500.
    int nT = 0;
    world.query<Transform>().each([&](Entity, Transform&) { ++nT; });
    assert(nT == 1500);
    std::printf("query<Transform> = %d\n", nT);

    // query<Transform, Velocity> matches only A -> 1000; integrate position += v.
    int nTV = 0;
    world.query<Transform, Velocity>().each([&](Entity, Transform& t, Velocity& vel) {
        t.position += vel.v;
        ++nTV;
    });
    assert(nTV == 1000);
    std::printf("query<Transform,Velocity> = %d\n", nTV);

    // chunks: sum via contiguous spans, and verify integration applied.
    size_t chunkTotal = 0;
    bool   integrated = true;
    world.query<Transform, Velocity>().chunks([&](std::span<Transform> ts, std::span<Velocity> vs) {
        chunkTotal += ts.size();
        for (size_t i = 0; i < ts.size(); ++i)
            if (ts[i].position.y != vs[i].v.y) integrated = false;   // y was 0, += 2
    });
    assert(chunkTotal == 1000);
    assert(integrated);

    // Spawn one, read it back through get().
    Entity e = world.spawn(Transform{ .position = glm::vec3(7, 8, 9) }, Velocity{ glm::vec3(0) });
    assert(world.has<Transform>(e) && world.has<Velocity>(e));
    assert(world.get<Transform>(e)->position.x == 7.0f);

    // Destroy it: stale handle invalidated, counts drop, others intact.
    world.destroy(e);
    assert(!world.alive(e));
    assert(world.get<Transform>(e) == nullptr);
    assert(world.size() == 1500);

    int nAfter = 0;
    world.query<Transform>().each([&](Entity, Transform&) { ++nAfter; });
    assert(nAfter == 1500);

    std::printf("ecs ok (entities=%zu)\n", world.size());
}
