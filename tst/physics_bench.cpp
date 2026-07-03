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
std::unique_ptr<phys::PhysicsWorld> makeFreefallWorld(int n, int substeps) {
    phys::WorldDef wd;
    wd.gravity = phys::Vec3(0, -9.81f, 0);
    wd.velocityIterations = 8;
    wd.substeps = substeps;
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

// Median wall time (seconds) of a single world->step() over `steps` measured steps, `reps` reps.
double benchRawStep(int n, int steps, int reps, int substeps) {
    double best = 1e18;
    for (int rep = 0; rep < reps; ++rep) {
        auto world = makeFreefallWorld(n, substeps);
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
    auto world = makeFreefallWorld(n, substeps);
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

int main() {
#ifdef NDEBUG
    const char* build = "Release/optimized";
#else
    const char* build = "Debug (unoptimized — expect much slower)";
#endif
    std::printf("physics step benchmark — build: %s\n", build);
    std::printf("scenario: free-falling spheres, no contacts (broadphase + integrate bound)\n");
    std::printf("backend: realtime, brute-force O(n^2) broadphase, substeps=1\n\n");

    std::printf("%8s | %12s | %12s | %14s | %16s\n",
                "bodies", "us/step", "ms/step", "steps/sec", "body-steps/sec");
    std::printf("---------+--------------+--------------+----------------+------------------\n");

    for (int n : { 64, 256, 1024, 4096 }) {
        const int steps = (n <= 1024) ? 200 : 60;
        const double perStep = benchRawStep(n, steps, 3, /*substeps=*/1);
        std::printf("%8d | %12.2f | %12.4f | %14.0f | %16.3e\n",
                    n, perStep * 1e6, perStep * 1e3, 1.0 / perStep, n / perStep);
    }

    std::printf("\nECS-bridge overhead (stepSystem + syncSystem via Schedule):\n");
    std::printf("%8s | %12s | %12s\n", "bodies", "us/step", "ms/step");
    std::printf("---------+--------------+--------------\n");
    for (int n : { 256, 1024, 4096 }) {
        const double perStep = benchBridgeStep(n, 100, /*substeps=*/1);
        std::printf("%8d | %12.2f | %12.4f\n", n, perStep * 1e6, perStep * 1e3);
    }

    std::printf("\nbenchmark done\n");
    return 0;
}
