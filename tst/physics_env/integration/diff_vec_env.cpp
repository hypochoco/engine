#include "harness/harness.h"
//
//  diff_vec_env.cpp
//  engine::tst / physics_env / integration
//
//  The CPU DiffVecEnv — the host counterpart of the GPU CudaVecEnv running the SAME templated diff ABA
//  + shared actuation/obs (diff/env_ops.h). Validates on the Mac (ENGINE_CUDA off) what the GPU path
//  can't be tested for here: obs dimensions/layout, that the PD servo actually actuates (holds the
//  humanoid up vs a limp zero-torque ragdoll), determinism, and per-env reset_masked. This is the
//  CPU half of the "CPU and CUDA are the same physics" unification (2026-07-08-cuda-port-code-review.md).
//

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_env/diff_vec_env.h"

using namespace engine;
using namespace engine::physics_env;

namespace {
EnvConfig humanoidCfg(physics::ActionMode mode, int substeps = 48) {
    EnvConfig c;
    c.articulation = physics::makeHumanoid();
    c.sim.substeps = substeps;
    c.sim.controlDt = physics::Real(1) / physics::Real(60);
    c.sim.actionMode = mode;
    c.sim.kp = physics::Real(150); c.sim.kd = physics::Real(15); c.sim.maxTorque = physics::Real(150);
    return c;
}
float rootYAfter(physics::ActionMode mode, int steps) {
    DiffVecEnv e(1, humanoidCfg(mode));
    e.reset(0);
    for (float& v : e.actions()) v = 0.0f;   // zero action: PD holds rest pose / Torque = limp ragdoll
    for (int i = 0; i < steps; ++i) e.step();
    return e.observations()[1];               // basePos.y
}
} // namespace

TST_CASE(physics_env, integration, diff_vec_env) {
    // --- dimensions + obs layout (humanoid: 14 bodies / 21 DOF ⇒ obsDim = 13 + 2*21 + 14 = 69) ---
    DiffVecEnv env(4, humanoidCfg(physics::ActionMode::PDTarget));
    TST_REQUIRE(env.actDim() == 21);
    TST_REQUIRE(env.obsDim() == 69);
    env.reset(0);
    const auto obs0 = env.observations();
    TST_APPROX(obs0[1], 0.99f, 0.05f);        // authored standing root height
    bool finite = true; for (float v : obs0) finite = finite && std::isfinite(v);
    TST_REQUIRE(finite);

    // --- actuation works: over a short (float-stable) horizon, PD-hold keeps the humanoid up while a
    //     zero-torque ragdoll sags onto the contact. (Long-horizon float stability is a separate
    //     tuning concern — the GPU kernel shares it; validated regime here matches cuda_forward.cpp.) ---
    const int H = 20;
    const float yPD  = rootYAfter(physics::ActionMode::PDTarget, H);
    const float yRag = rootYAfter(physics::ActionMode::Torque,   H);
    std::printf("diff_vec_env: obsDim=%zu  yPD=%.3f  yRagdoll=%.3f (after %d control steps)\n", env.obsDim(), yPD, yRag, H);
    TST_REQUIRE(std::isfinite(yPD) && std::isfinite(yRag));
    TST_REQUIRE_MSG(yPD > yRag, "PD servo should hold the humanoid higher than a limp ragdoll");

    // --- determinism: two identical runs are bit-identical (serial, deterministic float math) ---
    //  Kept in the float-stable regime (zero PD targets = hold): stiff PD (kp=150) + float + smoothed
    //  contact is stability-sensitive under actuation over long horizons — a property the GPU path
    //  shares and the sim1 layer manages (substeps / gains / divergence guard), not a wrapper bug.
    auto run = []() {
        DiffVecEnv e(2, humanoidCfg(physics::ActionMode::PDTarget));
        e.reset(0);
        for (float& v : e.actions()) v = 0.0f;   // hold rest pose (finite over the horizon)
        for (int i = 0; i < 20; ++i) e.step();
        const auto o = e.observations(); return std::vector<float>(o.begin(), o.end());
    };
    const auto r1 = run(), r2 = run();
    int nanCount = 0; double maxDiff = 0;
    for (size_t k = 0; k < r1.size(); ++k) {
        if (!std::isfinite(r1[k])) ++nanCount;
        maxDiff = std::max(maxDiff, std::fabs((double)r1[k] - (double)r2[k]));
    }
    std::printf("diff_vec_env: determinism nanCount=%d maxDiff=%.3e\n", nanCount, maxDiff);
    TST_REQUIRE_MSG(nanCount == 0, "PD-hold rollout should stay finite");
    bool same = (r1.size() == r2.size()); for (size_t k = 0; k < r1.size() && same; ++k) same = (r1[k] == r2[k]);
    TST_REQUIRE(same);

    // --- reset_masked resets only flagged envs ---
    DiffVecEnv m(2, humanoidCfg(physics::ActionMode::PDTarget));
    m.reset(0);
    for (float& v : m.actions()) v = 0.0f;     // hold (finite); envs drift up off the authored 0.99 init
    for (int i = 0; i < 20; ++i) m.step();
    const std::vector<float> before(m.observations().begin(), m.observations().end());
    TST_REQUIRE(std::fabs(before[m.obsDim() + 1] - 0.99f) > 0.02f);   // env1 has moved from init (non-trivial reset test)
    const uint8_t mask[2] = { 1, 0 };
    m.resetMasked(std::span<const uint8_t>(mask, 2), 0);
    const auto obs = m.observations();
    TST_APPROX(obs[1], 0.99f, 0.05f);          // env 0 reset to authored init
    bool env1Unchanged = true;
    for (size_t k = 0; k < m.obsDim(); ++k) env1Unchanged = env1Unchanged && (obs[m.obsDim() + k] == before[m.obsDim() + k]);
    TST_REQUIRE_MSG(env1Unchanged, "reset_masked must not touch unflagged envs");
    std::printf("diff_vec_env: determinism + reset_masked ok\n");
}
