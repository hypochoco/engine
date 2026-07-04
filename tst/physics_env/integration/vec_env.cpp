//
//  vec_env.cpp
//  engine::tst / physics_env / integration
//
//  Phase D2: a batch of humanoid envs stepped in parallel across a ThreadPool must be
//  bit-identical to stepping them serially (disjoint single-threaded worlds ⇒ deterministic), and
//  produce finite batched observations.
//

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_env/vec_env.h"
#include "harness/harness.h"

using namespace engine;

TST_CASE(physics_env, integration, vec_env_parallel_determinism) {
    physics_env::EnvConfig cfg;
    cfg.articulation = physics::makeHumanoid();
    cfg.sim.maxTorque = 60.0f;

    constexpr size_t N = 24;
    engine::core::ThreadPool pool;
    physics_env::VecEnv parallel(N, cfg, &pool);
    physics_env::VecEnv serial(N, cfg, nullptr);

    parallel.reset(2025);
    serial.reset(2025);

    std::mt19937 rng(555);
    std::uniform_real_distribution<float> d(-40.0f, 40.0f);

    float maxErr = 0.0f;
    for (int t = 0; t < 60; ++t) {
        // Same action batch fed to both (fill parallel's buffer, copy to serial's).
        for (float& v : parallel.actions()) v = d(rng);
        std::span<float> sa = serial.actions();
        const std::span<float> pa = parallel.actions();
        for (size_t k = 0; k < sa.size(); ++k) sa[k] = pa[k];

        parallel.step();
        serial.step();

        const auto po = parallel.observations();
        const auto so = serial.observations();
        for (size_t k = 0; k < po.size(); ++k) maxErr = std::max(maxErr, std::fabs(po[k] - so[k]));
    }

    std::printf("vec_env: N=%zu obsDim=%zu parallel-vs-serial maxErr=%.3e\n",
                N, parallel.obsDim(), maxErr);
    TST_REQUIRE(parallel.observations().size() == N * parallel.obsDim());
    for (float v : parallel.observations()) TST_REQUIRE(std::isfinite(v));
    TST_REQUIRE(maxErr == 0.0f);   // parallel batch == serial, bit-identical
}
