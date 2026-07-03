#include "harness/harness.h"
//
//  physics_bench.cpp
//  engine::tst
//
//  Headless performance baseline for the physics step, captured BEFORE the Phase-2 scaling
//  work (GJK/EPA + SAP/BVH broadphase). Reports step throughput vs body count for the current
//  brute-force O(n²) broadphase, plus the ECS-bridge overhead (step + sync via the scheduler).
//
//  NOTE: absolute numbers depend on hardware, CPU power/thermal state, build type, and other
//  load — treat them as a relative baseline for before/after comparisons on the SAME machine,
//  not as portable figures. Build type is printed below.
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "engine/core/math/transform.h"
#include "engine/core/threading/thread_pool.h"
#include "engine/ecs/ecs.h"
#include "engine/physics/world.h"
#include "engine/physics_ecs/components.h"
#include "engine/physics_ecs/systems.h"

using namespace engine;
namespace phys = engine::physics;
using Clock = std::chrono::steady_clock;

namespace {

// A grid of free-falling spheres far above the plane so nothing contacts within the run: this
// isolates the broadphase (O(n²) narrowphase) + integration cost — the pre-scaling bottleneck.
std::unique_ptr<phys::PhysicsWorld> makeFreefallWorld(int n, int substeps, phys::BroadphaseKind bp,
                                                     engine::core::ThreadPool* pool = nullptr) {
    phys::WorldDef wd;
    wd.gravity = phys::Vec3(0, -9.81f, 0);
    wd.velocityIterations = 8;
    wd.substeps = substeps;
    wd.broadphase = bp;
    wd.threadPool = pool;
    wd.parallelThreshold = 1024;
    auto world = phys::createPhysicsWorld(phys::Backend::Realtime, wd);

    phys::BodyDef plane;
    plane.type = phys::BodyType::Static;
    plane.collider.type = phys::ColliderDesc::Type::Plane;
    plane.collider.plane = phys::Plane{ phys::Vec3(0, 1, 0), -1000.0f };   // far below
    world->createBody(plane);

    const int side = static_cast<int>(std::ceil(std::cbrt(static_cast<double>(n))));
    const float spacing = 1.5f;   // > 2r, no initial overlap
    int made = 0;
    for (int z = 0; z < side && made < n; ++z)
        for (int y = 0; y < side && made < n; ++y)
            for (int x = 0; x < side && made < n; ++x, ++made) {
                phys::BodyDef b;
                b.type = phys::BodyType::Dynamic;
                b.mass = 1.0f;
                b.collider.type = phys::ColliderDesc::Type::Sphere;
                b.collider.sphere = phys::Sphere{ 0.5f };
                b.position = phys::Vec3(x * spacing, y * spacing, z * spacing);
                world->createBody(b);
            }
    return world;
}

// A dense block of overlapping spheres resting on a plane — many contacts every step, so the
// narrowphase + contact solver dominate (unlike the free-fall broadphase-bound scenario).
std::unique_ptr<phys::PhysicsWorld> makePileWorld(int n, engine::core::ThreadPool* pool) {
    phys::WorldDef wd;
    wd.gravity = phys::Vec3(0, -9.81f, 0);
    wd.velocityIterations = 8;
    wd.substeps = 1;
    wd.broadphase = phys::BroadphaseKind::UniformGrid;
    wd.threadPool = pool;
    wd.parallelThreshold = 1024;
    auto world = phys::createPhysicsWorld(phys::Backend::Realtime, wd);

    phys::BodyDef plane;
    plane.type = phys::BodyType::Static;
    plane.collider.type = phys::ColliderDesc::Type::Plane;
    plane.collider.plane = phys::Plane{ phys::Vec3(0, 1, 0), 0.0f };
    plane.material.friction = 0.8f;
    world->createBody(plane);

    const int side = static_cast<int>(std::ceil(std::cbrt(static_cast<double>(n))));
    const float spacing = 0.9f;   // < 2r=1.0 -> neighbours overlap -> dense contacts
    int made = 0;
    for (int z = 0; z < side && made < n; ++z)
        for (int y = 0; y < side && made < n; ++y)
            for (int x = 0; x < side && made < n; ++x, ++made) {
                phys::BodyDef b;
                b.type = phys::BodyType::Dynamic;
                b.mass = 1.0f;
                b.collider.type = phys::ColliderDesc::Type::Sphere;
                b.collider.sphere = phys::Sphere{ 0.5f };
                b.material.friction = 0.8f;
                b.position = phys::Vec3(x * spacing, 0.6f + y * spacing, z * spacing);
                world->createBody(b);
            }
    return world;
}

// Median wall time (seconds) of a single world->step() over `steps` measured steps, `reps` reps.
double benchRawStep(int n, int steps, int reps, int substeps, phys::BroadphaseKind bp) {
    double best = 1e18;
    for (int rep = 0; rep < reps; ++rep) {
        auto world = makeFreefallWorld(n, substeps, bp);
        const float dt = 1.0f / 120.0f;
        for (int i = 0; i < 5; ++i) world->step(dt);   // warm up
        const auto t0 = Clock::now();
        for (int i = 0; i < steps; ++i) world->step(dt);
        const auto t1 = Clock::now();
        const double perStep = std::chrono::duration<double>(t1 - t0).count() / steps;
        best = std::min(best, perStep);
    }
    return best;   // best-of-reps single-step time (least noisy)
}

// ECS-bridge step: scheduler runs stepSystem (physics) + syncSystem (poses -> Transform).
double benchBridgeStep(int n, int steps, int substeps) {
    auto world = makeFreefallWorld(n, substeps, phys::BroadphaseKind::UniformGrid);
    ecs::World ecsWorld;
    // Body indices 1..n are the dynamic spheres (index 0 is the plane).
    for (int i = 0; i < n; ++i)
        ecsWorld.spawn(engine::Transform{},
                       physics_ecs::RigidBody{ phys::BodyHandle{ static_cast<uint32_t>(i + 1), 0 } });
    ecsWorld.setResource(physics_ecs::PhysicsWorldRef{ world.get() });
    ecsWorld.setResource(physics_ecs::FixedStep{ 1.0f / 120.0f });
    ecs::Schedule sim;
    sim.add("physics.step", physics_ecs::stepSystem);
    sim.add("physics.sync", physics_ecs::syncSystem);

    for (int i = 0; i < 5; ++i) sim.run(ecsWorld);   // warm up
    const auto t0 = Clock::now();
    for (int i = 0; i < steps; ++i) sim.run(ecsWorld);
    const auto t1 = Clock::now();
    return std::chrono::duration<double>(t1 - t0).count() / steps;
}

} // namespace

TST_CASE(physics, benchmark, step) {
#ifdef NDEBUG
    const char* build = "Release/optimized";
#else
    const char* build = "Debug (unoptimized — expect much slower)";
#endif
    std::printf("physics step benchmark — build: %s\n", build);
    std::printf("scenario: free-falling spheres, no contacts (broadphase + integrate bound)\n\n");

    struct Bp { const char* name; phys::BroadphaseKind kind; };
    for (const Bp bp : { Bp{ "sweep-and-prune", phys::BroadphaseKind::SweepAndPrune },
                         Bp{ "uniform-grid",    phys::BroadphaseKind::UniformGrid } }) {
        std::printf("broadphase: %s\n", bp.name);
        std::printf("%8s | %12s | %12s | %14s | %16s\n",
                    "bodies", "us/step", "ms/step", "steps/sec", "body-steps/sec");
        std::printf("---------+--------------+--------------+----------------+------------------\n");
        for (int n : { 256, 1024, 4096, 16384, 65536 }) {
            const int steps = (n <= 1024) ? 200 : (n <= 16384 ? 60 : 20);
            const double perStep = benchRawStep(n, steps, 3, /*substeps=*/1, bp.kind);
            std::printf("%8d | %12.2f | %12.4f | %14.0f | %16.3e\n",
                        n, perStep * 1e6, perStep * 1e3, 1.0 / perStep, n / perStep);
        }
        std::printf("\n");
    }

    std::printf("ECS-bridge overhead (uniform-grid; stepSystem + syncSystem via Schedule):\n");
    std::printf("%8s | %12s | %12s\n", "bodies", "us/step", "ms/step");
    std::printf("---------+--------------+--------------\n");
    for (int n : { 1024, 4096, 16384 }) {
        const double perStep = benchBridgeStep(n, 60, /*substeps=*/1);
        std::printf("%8d | %12.2f | %12.4f\n", n, perStep * 1e6, perStep * 1e3);
    }

    // Parallel worlds: M independent envs (ML throughput / "parallel simulations" milestone).
    {
        const int   M = 64;      // worlds
        const int   K = 1024;    // bodies per world
        const int   S = 60;      // steps per sweep
        const float dt = 1.0f / 120.0f;

        std::vector<std::unique_ptr<phys::PhysicsWorld>> worlds;
        for (int i = 0; i < M; ++i)
            worlds.push_back(makeFreefallWorld(K, 1, phys::BroadphaseKind::UniformGrid));
        for (auto& w : worlds) for (int s = 0; s < 5; ++s) w->step(dt);   // warm up

        auto t0 = Clock::now();
        for (int i = 0; i < M; ++i) for (int s = 0; s < S; ++s) worlds[i]->step(dt);
        const double serial = std::chrono::duration<double>(Clock::now() - t0).count();

        engine::core::ThreadPool pool;
        t0 = Clock::now();
        pool.parallelFor(static_cast<std::size_t>(M),
                         [&](std::size_t i) { for (int s = 0; s < S; ++s) worlds[i]->step(dt); }, 1);
        const double parallel = std::chrono::duration<double>(Clock::now() - t0).count();

        const double bodySteps = double(M) * K * S;
        std::printf("\nparallel worlds: %d worlds x %d bodies x %d steps  (workers=%u)\n",
                    M, K, S, pool.workerCount());
        std::printf("  serial   : %8.2f ms   (%.3e body-steps/s)\n", serial * 1e3, bodySteps / serial);
        std::printf("  parallel : %8.2f ms   (%.3e body-steps/s)\n", parallel * 1e3, bodySteps / parallel);
        std::printf("  speedup  : %.2fx\n", serial / parallel);
    }

    // Intra-world parallelism: ONE dense contact pile, serial vs pooled step (parallel
    // integration + narrowphase; solver + broadphase-sort still serial).
    {
        const int   N = 32768;
        const int   S = 30;
        const float dt = 1.0f / 120.0f;
        engine::core::ThreadPool pool;

        auto run = [&](engine::core::ThreadPool* p) {
            auto w = makePileWorld(N, p);
            for (int s = 0; s < 5; ++s) w->step(dt);   // warm up (also lets the pile settle)
            const auto t0 = Clock::now();
            for (int s = 0; s < S; ++s) w->step(dt);
            return std::chrono::duration<double>(Clock::now() - t0).count() / S;
        };
        const double serial = run(nullptr);
        const double parallel = run(&pool);
        std::printf("\nintra-world (dense pile, %d bodies, workers=%u): "
                    "serial %.2f ms/step, parallel %.2f ms/step, speedup %.2fx\n",
                    N, pool.workerCount(), serial * 1e3, parallel * 1e3, serial / parallel);
        std::printf("  (parallel stages: integration + narrowphase + colored solver)\n");
    }

    // Intra-world, sparse: ONE large free-fall world, serial vs pooled (parallel entry sort +
    // integration — the broadphase-bound regime).
    {
        const int   N = 65536;
        const int   S = 20;
        const float dt = 1.0f / 120.0f;
        engine::core::ThreadPool pool;

        auto run = [&](engine::core::ThreadPool* p) {
            auto w = makeFreefallWorld(N, 1, phys::BroadphaseKind::UniformGrid, p);
            for (int s = 0; s < 5; ++s) w->step(dt);
            const auto t0 = Clock::now();
            for (int s = 0; s < S; ++s) w->step(dt);
            return std::chrono::duration<double>(Clock::now() - t0).count() / S;
        };
        const double serial = run(nullptr);
        const double parallel = run(&pool);
        std::printf("\nintra-world (sparse free-fall, %d bodies, workers=%u): "
                    "serial %.2f ms/step, parallel %.2f ms/step, speedup %.2fx\n",
                    N, pool.workerCount(), serial * 1e3, parallel * 1e3, serial / parallel);
        std::printf("  (parallel stages: grid entry sort + integration)\n");
    }

    std::printf("\nbenchmark done\n");
}
