//
//  vec_env.cpp
//  engine::tst / physics_env / benchmark
//
//  Phase D3: env-steps/sec for a batch of humanoid envs stepped across the ThreadPool, at
//  N ∈ {1, 64, 1024}. Prints throughput (not pass/fail); run in an optimized build:
//    ./build/tst/benchmarks
//

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_env/vec_env.h"
#include "harness/harness.h"

using namespace engine;

TST_CASE(physics_env, benchmark, vec_env_throughput) {
    physics_env::EnvConfig cfg;
    cfg.articulation = physics::makeHumanoid();
    cfg.sim.maxTorque = 60.0f;

    engine::core::ThreadPool pool;
    constexpr int kSteps = 100;
    std::printf("VecEnv humanoid throughput (%u workers, %d control steps each):\n",
                pool.workerCount() + 1, kSteps);

    for (size_t N : { size_t(1), size_t(64), size_t(1024) }) {
        physics_env::VecEnv vec(N, cfg, &pool);
        vec.reset(1);

        std::mt19937 rng(7);
        std::uniform_real_distribution<float> d(-30.0f, 30.0f);

        const auto t0 = std::chrono::steady_clock::now();
        for (int t = 0; t < kSteps; ++t) {
            for (float& v : vec.actions()) v = d(rng);
            vec.step();
        }
        const auto t1 = std::chrono::steady_clock::now();

        const double secs = std::chrono::duration<double>(t1 - t0).count();
        const double envSteps = static_cast<double>(N) * kSteps;
        std::printf("  N=%5zu  %8.1f ms  %10.0f env-steps/s  (%.3f ms/batched-step)\n",
                    N, secs * 1e3, envSteps / secs, (secs / kSteps) * 1e3);
    }
}
