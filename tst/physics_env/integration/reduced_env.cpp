//
//  reduced_env.cpp
//  engine::tst / physics_env / integration
//
//  Phase E3: the same humanoid Environment/VecEnv running on the reduced-coordinate backend
//  (Backend::Reduced) with NO env-layer changes — only EnvConfig.backend flips. Validates that the
//  obs/action API is backend-agnostic, the reduced humanoid env is finite/bounded under random
//  torques, and determinism holds (single-env repeatable; VecEnv parallel == serial, bit-identical).
//

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_env/environment.h"
#include "engine/physics_env/vec_env.h"
#include "harness/harness.h"

using namespace engine;

namespace {
physics_env::EnvConfig reducedHumanoid() {
    physics_env::EnvConfig c;
    c.articulation = physics::makeHumanoid();
    c.backend = physics::Backend::Reduced;
    c.substeps = 24;        // reduced ABA needs a finer step than maximal PGS under strong torque
    c.maxTorque = 40.0f;
    return c;
}
}

TST_CASE(physics_env, integration, reduced_env_finite_and_deterministic) {
    physics_env::Environment a(reducedHumanoid());
    physics_env::Environment b(reducedHumanoid());
    // Same actDim/obsDim as the maximal backend (API unchanged): 21 / 53 for the humanoid.
    TST_REQUIRE(a.actDim() == 21);
    TST_REQUIRE(a.defaultObsDim() == 53);

    a.reset(11);
    b.reset(11);
    std::mt19937 rng(2024);
    std::uniform_real_distribution<float> d(-15.0f, 15.0f);
    std::vector<float> action(a.actDim());
    std::vector<float> oa(a.defaultObsDim()), ob(b.defaultObsDim());

    float maxErr = 0;
    for (int i = 0; i < 200; ++i) {
        for (float& v : action) v = d(rng);
        a.setAction(action); a.step();
        b.setAction(action); b.step();
        a.packDefaultObs(oa); b.packDefaultObs(ob);
        for (size_t k = 0; k < oa.size(); ++k) maxErr = std::max(maxErr, std::fabs(oa[k] - ob[k]));
    }
    for (float v : oa) TST_REQUIRE(std::isfinite(v));
    const auto p = a.rootPose().position;
    std::printf("reduced_env: rootFinal=(%.3f,%.3f,%.3f) determinismMaxErr=%.3e\n", p.x, p.y, p.z, maxErr);
    TST_REQUIRE(std::fabs(p.x) < 5.0f && std::fabs(p.z) < 5.0f && p.y > -1.0f && p.y < 3.0f);  // bounded
    TST_REQUIRE(maxErr == 0.0f);                                                                // deterministic
}

TST_CASE(physics_env, integration, reduced_vecenv_parallel_determinism) {
    physics_env::EnvConfig cfg = reducedHumanoid();
    constexpr size_t N = 16;
    engine::core::ThreadPool pool;
    physics_env::VecEnv parallel(N, cfg, &pool);
    physics_env::VecEnv serial(N, cfg, nullptr);
    parallel.reset(7);
    serial.reset(7);

    std::mt19937 rng(321);
    std::uniform_real_distribution<float> d(-15.0f, 15.0f);
    float maxErr = 0;
    for (int t = 0; t < 40; ++t) {
        for (float& v : parallel.actions()) v = d(rng);
        std::span<float> sa = serial.actions();
        const std::span<float> pa = parallel.actions();
        for (size_t k = 0; k < sa.size(); ++k) sa[k] = pa[k];
        parallel.step(); serial.step();
        const auto po = parallel.observations(); const auto so = serial.observations();
        for (size_t k = 0; k < po.size(); ++k) maxErr = std::max(maxErr, std::fabs(po[k] - so[k]));
    }
    std::printf("reduced_vecenv: N=%zu parallel-vs-serial maxErr=%.3e\n", N, maxErr);
    for (float v : parallel.observations()) TST_REQUIRE(std::isfinite(v));
    TST_REQUIRE(maxErr == 0.0f);
}
