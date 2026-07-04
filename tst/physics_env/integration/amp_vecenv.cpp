//
//  amp_vecenv.cpp
//  engine::tst / physics_env / integration
//
//  Validate the 28-DOF AMP rig (makeAMPHumanoid) through the PRODUCTION env — `physics_env::
//  Environment`/`VecEnv` over a real PhysicsWorld backend (the PPO control path, distinct from the
//  differentiable engine). Confirms the generic actuator/obs layout sizes correctly for the richer
//  rig (actDim 28, obsDim 84) on both backends, the rig is sound (finite passive + gently-actuated
//  rollout), and the parallel batch is bit-identical to serial (the determinism the trainer relies on).
//
//  Note: like the reduced humanoid, strong torques at the coarse default substeps=8 diverge — a
//  training-config concern (finer substeps / bounded actions), not a rig bug; passive is stable at 8.
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

namespace {
physics_env::EnvConfig ampConfig(physics::Backend backend, int substeps) {
    physics_env::EnvConfig cfg;
    cfg.articulation = physics::makeAMPHumanoid();
    cfg.sim.backend = backend;
    cfg.sim.substeps = substeps;
    cfg.sim.maxTorque = 60.0f;
    return cfg;
}
constexpr size_t kAmpAct = 28;                 // 8 ball(3) = 24 + 4 hinge = 28 actuated DOF (wrists fixed)
constexpr size_t kAmpObs = 13 + 2 * 28 + 15;   // root7 + twist6 + q[28] + qd[28] + contacts[15] = 84

bool rollout(physics_env::Environment& env, float torque, int substepsUnused, unsigned seed) {
    (void)substepsUnused;
    std::vector<float> a(env.actDim(), 0.0f), ob(env.defaultObsDim());
    std::mt19937 rng(seed); std::uniform_real_distribution<float> d(-torque, torque);
    bool finite = true;
    for (int t = 0; t < 60 && finite; ++t) {
        for (float& x : a) x = (torque > 0.0f) ? d(rng) : 0.0f;
        env.setAction(a);
        env.step();
        env.packDefaultObs(ob);
        for (float v : ob) finite = finite && std::isfinite(v);
    }
    return finite;
}
}

// (1) Correct dims + a sound rig on both production backends: finite passive AND finite gently-actuated.
TST_CASE(physics_env, integration, amp_vecenv_dims_and_rollout) {
    for (physics::Backend b : { physics::Backend::Realtime, physics::Backend::Reduced }) {
        physics_env::Environment envDims(ampConfig(b, 8));
        std::printf("amp_env backend=%d actDim=%zu obsDim=%zu\n", static_cast<int>(b), envDims.actDim(), envDims.defaultObsDim());
        TST_REQUIRE(envDims.actDim() == kAmpAct);
        TST_REQUIRE(envDims.defaultObsDim() == kAmpObs);

        physics_env::Environment passive(ampConfig(b, 8));   passive.reset(7);
        TST_REQUIRE(rollout(passive, 0.0f, 8, 1));           // passive: rig is sound at the default substeps
        physics_env::Environment driven(ampConfig(b, 24));   driven.reset(7);
        TST_REQUIRE(rollout(driven, 6.0f, 24, 2));           // gently actuated at finer substeps: stays finite
    }
}

// (2) Parallel VecEnv of AMP envs == serial, bit-identical (batched-rollout determinism).
TST_CASE(physics_env, integration, amp_vecenv_parallel_determinism) {
    const physics_env::EnvConfig cfg = ampConfig(physics::Backend::Reduced, 24);
    constexpr size_t N = 16;
    engine::core::ThreadPool pool;
    physics_env::VecEnv par(N, cfg, &pool);
    physics_env::VecEnv ser(N, cfg, nullptr);
    par.reset(2025); ser.reset(2025);

    std::mt19937 rng(9); std::uniform_real_distribution<float> d(-2.0f, 2.0f);
    float maxErr = 0.0f;
    for (int t = 0; t < 40; ++t) {
        for (float& v : par.actions()) v = d(rng);
        std::span<float> sa = ser.actions(); const std::span<float> pa = par.actions();
        for (size_t k = 0; k < sa.size(); ++k) sa[k] = pa[k];
        par.step(); ser.step();
        const auto po = par.observations(); const auto so = ser.observations();
        for (size_t k = 0; k < po.size(); ++k) maxErr = std::max(maxErr, std::fabs(po[k] - so[k]));
    }
    std::printf("amp_vecenv: N=%zu actDim=%zu obsDim=%zu parallel-vs-serial maxErr=%.3e\n",
                N, par.actDim(), par.obsDim(), maxErr);
    TST_REQUIRE(par.actDim() == kAmpAct && par.obsDim() == kAmpObs);
    for (float v : par.observations()) TST_REQUIRE(std::isfinite(v));
    TST_REQUIRE(maxErr == 0.0f);
}
