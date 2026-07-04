//
//  reduced_env.cpp
//  engine::tst / physics_env / benchmark
//
//  Phase E3: humanoid env throughput, reduced vs maximal backend, across a batch on the ThreadPool.
//  Same substeps for a per-step cost comparison (the reduced ABA does CRBA + a dense H⁻¹ for the
//  contact solve; the maximal backend does sequential-impulse PGS). Run in an optimized build:
//    ./build/tst/benchmarks
//

#include <chrono>
#include <cstdio>
#include <random>

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_env/vec_env.h"
#include "harness/harness.h"

using namespace engine;

namespace {
void run(const char* label, physics::Backend backend, engine::core::ThreadPool& pool) {
    physics_env::EnvConfig cfg;
    cfg.articulation = physics::makeHumanoid();
    cfg.backend = backend;
    cfg.substeps = 16;
    cfg.maxTorque = 40.0f;
    constexpr int kSteps = 60;
    std::printf("  %-9s:", label);
    for (size_t N : { size_t(1), size_t(64), size_t(1024) }) {
        physics_env::VecEnv vec(N, cfg, &pool);
        vec.reset(1);
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> d(-8.0f, 8.0f);
        const auto t0 = std::chrono::steady_clock::now();
        for (int t = 0; t < kSteps; ++t) { for (float& v : vec.actions()) v = d(rng); vec.step(); }
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("  N=%zu %.0f env-steps/s", N, (double(N) * kSteps) / secs);
    }
    std::printf("\n");
}
}

TST_CASE(physics_env, benchmark, reduced_vs_maximal_throughput) {
    engine::core::ThreadPool pool;
    std::printf("Humanoid env throughput (%u workers, substeps=16):\n", pool.workerCount() + 1);
    run("maximal", physics::Backend::Realtime, pool);
    run("reduced", physics::Backend::Reduced, pool);
}
