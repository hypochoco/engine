//
//  environment.cpp
//  engine::tst / physics_env / unit
//
//  Phase D1: the headless RL Environment over a humanoid. Checks action/obs dimensions, that a
//  reset + random-torque rollout stays finite/bounded, and that the env is deterministic (two
//  envs fed the same action stream produce bit-identical observations).
//

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_env/environment.h"
#include "harness/harness.h"

using namespace engine;

namespace {

physics_env::EnvConfig humanoidEnv() {
    physics_env::EnvConfig c;
    c.articulation = physics::makeHumanoid();
    c.sim.maxTorque = 60.0f;
    return c;
}

// Deterministic pseudo-random action stream (fixed seed ⇒ reproducible).
void fillRandomAction(std::mt19937& rng, std::vector<float>& a, float scale) {
    std::uniform_real_distribution<float> d(-scale, scale);
    for (float& v : a) v = d(rng);
}

} // namespace

TST_CASE(physics_env, unit, env_dims) {
    physics_env::Environment env(humanoidEnv());
    // Humanoid actuated DOFs: 5 ball joints (waist + 2 shoulders + 2 hips) ×3 + 6 hinges
    // (2 elbows + 2 knees + 2 ankles) ×1 = 21. Fixed chest/neck contribute 0.
    TST_REQUIRE(env.actDim() == 21);
    // Default obs is DOF-complete: root pose(7) + twist(6) + positions[6 hinge + 5 ball×3 = 21]
    // + velocities[21] + contact flags[14] = 69.
    TST_REQUIRE(env.defaultObsDim() == 69);

    env.reset(0);
    std::vector<float> obs(env.defaultObsDim());
    env.packDefaultObs(obs);
    for (float v : obs) TST_REQUIRE(std::isfinite(v));
    // Root (pelvis) starts near its authored standing height (~0.99).
    TST_REQUIRE(std::fabs(env.rootPose().position.y - 0.99f) < 0.05f);
    std::printf("env_dims: actDim=%zu obsDim=%zu rootY=%.3f\n",
                env.actDim(), env.defaultObsDim(), env.rootPose().position.y);
}

TST_CASE(physics_env, unit, env_random_rollout_bounded) {
    physics_env::Environment env(humanoidEnv());
    env.reset(1);
    std::mt19937 rng(1234);
    std::vector<float> action(env.actDim());
    std::vector<float> obs(env.defaultObsDim());

    for (int i = 0; i < 300; ++i) {
        fillRandomAction(rng, action, 40.0f);
        env.setAction(action);
        env.step();
    }
    env.packDefaultObs(obs);
    for (float v : obs) TST_REQUIRE(std::isfinite(v));
    const auto p = env.rootPose().position;
    std::printf("random_rollout: root=(%.3f, %.3f, %.3f)\n", p.x, p.y, p.z);
    // Internal torques don't move the COM; gravity + contacts keep it local and above the floor.
    TST_REQUIRE(std::fabs(p.x) < 5.0f && std::fabs(p.z) < 5.0f);
    TST_REQUIRE(p.y > -0.5f && p.y < 3.0f);
}

TST_CASE(physics_env, unit, env_deterministic) {
    physics_env::Environment a(humanoidEnv());
    physics_env::Environment b(humanoidEnv());
    a.reset(7);
    b.reset(7);

    std::mt19937 rng(99);
    std::vector<float> action(a.actDim());
    std::vector<float> oa(a.defaultObsDim()), ob(b.defaultObsDim());

    float maxErr = 0.0f;
    for (int i = 0; i < 200; ++i) {
        fillRandomAction(rng, action, 30.0f);
        a.setAction(action);  a.step();
        b.setAction(action);  b.step();
        a.packDefaultObs(oa); b.packDefaultObs(ob);
        for (size_t k = 0; k < oa.size(); ++k) maxErr = std::max(maxErr, std::fabs(oa[k] - ob[k]));
    }
    std::printf("env_deterministic: max obs divergence = %.3e\n", maxErr);
    TST_REQUIRE(maxErr == 0.0f);   // identical config + action stream ⇒ bit-identical rollout
}
