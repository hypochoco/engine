#include "harness/harness.h"
//
//  box_stability.cpp
//  engine::tst / physics / benchmark
//
//  Scaling benchmark for the REALTIME contact solver on a contact-heavy BOX scene (a deep box pile),
//  the regime the stability work targets (split-impulse + contact warm-starting). Reports ms/step vs
//  body count up to ~100k, a phase breakdown (broadphase / narrowphase / solve — via engine::prof
//  zones, active in profiling builds), contact count, and a stability proxy (mean/max speed after a
//  warmup settle: a stable pile trends to ~0; energy injection keeps it lively). This is the
//  before/after yardstick for the fixes.
//
//  Absolute numbers are hardware/build/thermal dependent — same-machine before/after only.
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "engine/core/profile/profile.h"
#include "engine/core/threading/thread_pool.h"
#include "engine/physics/world.h"

using namespace engine;
namespace phys = engine::physics;
using Clock = std::chrono::steady_clock;

namespace {

// A deep pile of unit boxes on a ground plane: a cube-ish grid with small gaps so they settle onto
// each other into a lattice of resting box-box contacts (the tall-stack / dense-contact regime).
std::unique_ptr<phys::PhysicsWorld> makeBoxPile(int n, engine::core::ThreadPool* pool) {
    phys::WorldDef wd;
    wd.gravity = phys::Vec3(0, -18.0f, 0);   // match the arena feel
    wd.substeps = 4;
    wd.velocityIterations = 8;
    wd.linearDamping = 0.05f;
    wd.angularDamping = 0.1f;
    wd.broadphase = phys::BroadphaseKind::UniformGrid;
    wd.threadPool = pool;
    wd.parallelThreshold = 1024;
    if (const char* s = std::getenv("PHYS_TGS"))
        wd.contactSolver = std::atoi(s) ? phys::ContactSolver::TGSSoft : phys::ContactSolver::SequentialImpulse;
    if (const char* s = std::getenv("PHYS_SUBSTEPS")) wd.substeps = std::atoi(s);
    if (const char* s = std::getenv("PHYS_BAUM")) wd.solver.contactBaumgarte = std::atof(s);
    if (const char* s = std::getenv("PHYS_SLOP")) wd.solver.contactSlop = std::atof(s);
    auto world = phys::createPhysicsWorld(phys::Backend::Realtime, wd);

    phys::BodyDef plane;
    plane.type = phys::BodyType::Static;
    plane.collider.type = phys::ColliderDesc::Type::Plane;
    plane.collider.plane = phys::Plane{ phys::Vec3(0, 1, 0), 0.0f };
    plane.material.friction = 0.8f;
    world->createBody(plane);

    const int side = static_cast<int>(std::ceil(std::cbrt(static_cast<double>(n))));
    const float sp = 1.05f;   // 5 cm gaps → boxes settle onto each other (dense stacked contacts)
    int made = 0;
    for (int y = 0; y < side && made < n; ++y)
        for (int z = 0; z < side && made < n; ++z)
            for (int x = 0; x < side && made < n; ++x, ++made) {
                phys::BodyDef b;
                b.type = phys::BodyType::Dynamic; b.mass = 1.0f;
                b.collider.type = phys::ColliderDesc::Type::Box;
                b.collider.box = phys::Box{ phys::Vec3(0.5f) };
                b.material.friction = 0.8f;
                // slight per-column jitter so it's not a perfectly-aligned singular lattice
                const float jx = 0.01f * ((made * 13 % 7) - 3);
                b.position = phys::Vec3(x * sp + jx, 0.5f + y * sp, z * sp);
                world->createBody(b);
            }
    return world;
}

// Mean & max linear speed over all dynamic bodies (index 1..n; 0 is the plane).
void speedStats(const phys::PhysicsWorld& w, int n, double& mean, double& mx) {
    const auto v = w.linearVelocities();
    double s = 0; mx = 0;
    for (int i = 1; i <= n; ++i) {
        const double sp = glm::length(v[i]);
        s += sp; mx = std::max(mx, sp);
    }
    mean = s / std::max(1, n);
}

} // namespace

TST_CASE(physics, benchmark, box_pile_scaling) {
#ifdef NDEBUG
    const char* build = "Release/optimized";
#else
    const char* build = "Debug (unoptimized)";
#endif
    std::printf("box pile stability benchmark — build: %s, profiling=%s\n",
                build, prof::enabled() ? "ON (phase breakdown available)" : "OFF");
    std::printf("scenario: deep unit-box pile on a plane (substeps=4, velIter=8), pooled\n\n");

    engine::core::ThreadPool pool;
    const float dt = 1.0f / 60.0f;

    std::printf("%9s | %10s | %10s | %12s | %12s | %12s\n",
                "bodies", "ms/step", "contacts", "settle ms", "meanSpeed", "maxSpeed");
    std::printf("----------+------------+------------+--------------+--------------+--------------\n");

    for (int n : { 1000, 10000, 50000, 100000 }) {
        auto w = makeBoxPile(n, &pool);

        // Settle warmup (partial): lets the pile build its contact network before timing.
        const auto ts0 = Clock::now();
        constexpr int kWarm = 30;
        for (int s = 0; s < kWarm; ++s) w->step(dt);
        const double settleMs = std::chrono::duration<double, std::milli>(Clock::now() - ts0).count() / kWarm;

        // Timed steps.
        constexpr int kTimed = 20;
        const auto t0 = Clock::now();
        for (int s = 0; s < kTimed; ++s) w->step(dt);
        const double msStep = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / kTimed;

        double meanV, maxV; speedStats(*w, n, meanV, maxV);
        const size_t contacts = w->contacts().size();
        std::printf("%9d | %10.3f | %10zu | %12.3f | %12.4f | %12.4f\n",
                    n, msStep, contacts, settleMs, meanV, maxV);
    }

    // Phase breakdown at 100k (needs profiling build — RelWithDebInfo/Debug; no-op in Release).
    if constexpr (prof::enabled()) {
        auto w = makeBoxPile(100000, &pool);
        for (int s = 0; s < 30; ++s) w->step(dt);   // settle
        prof::reset();
        for (int s = 0; s < 20; ++s) w->step(dt);   // measured
        const prof::Report r = prof::report();
        std::printf("\nphase breakdown @ 100k (per-substep avg; ×4 substeps/step):\n");
        for (const auto& z : r.zones)
            if (z.name.rfind("phys.", 0) == 0)
                std::printf("  %-18s avg %7.3f ms  p95 %7.3f  (calls %llu)\n",
                            z.name.c_str(), z.avgMs, z.p95Ms, static_cast<unsigned long long>(z.calls));
    } else {
        std::printf("\n(build with ENGINE_PROFILING — RelWithDebInfo — for the phase breakdown)\n");
    }

    std::printf("\nbox pile benchmark done\n");
}
