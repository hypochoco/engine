//
//  backend_fidelity.cpp
//  engine::tst / physics_env / integration
//
//  Cross-backend fidelity guard: the reduced-coordinate PGS backend (Backend::Reduced) and the
//  diff-ABA smoothed-contact backend (DiffVecEnv) run the SAME humanoid via the SAME EnvConfig /
//  action / obs contract, so a policy trained on one is meaningful on the other and the diff-ABA
//  visualizer looks right. Companion to physics/integration/diff_validation.cpp, which already pins
//  PASSIVE, CONTACTLESS dynamics parity (diff_matches_reduced_backend). This file targets the two
//  places the backends currently DIVERGE — CONTACT and JOINT LIMITS — found in the 2026-07-09
//  fidelity deep dive (sim-1 notes/investigations/2026-07-09-engine-backend-fidelity.md).
//
//  Two of these are CHARACTERIZATION tests: they pin the *current* (buggy) divergence so it can't
//  silently grow and so the suite stays green while the fix is under review. Each documents the
//  TARGET invariant it should assert once the engine fix lands — when a fix shrinks the divergence
//  the characterization will start failing, which is the signal to tighten it to strict parity.
//

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/dynamics/articulation.h"
#include "engine/physics/diff/from_articulation.h"
#include "engine/physics_env/environment.h"
#include "engine/physics_env/diff_vec_env.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics_env;

namespace {
EnvConfig humanoidCfg() {
    EnvConfig c;
    c.articulation = physics::makeHumanoid();
    c.sim.substeps = 48;
    c.sim.controlDt = physics::Real(1) / physics::Real(60);
    c.sim.actionMode = physics::ActionMode::PDTarget;
    c.sim.kp = 150; c.sim.kd = 15; c.sim.maxTorque = 150;
    c.sim.backend = physics::Backend::Reduced;   // selects the reduced world for Environment/VecEnv
    return c;
}
constexpr int kNdof = 21;
float maxJointSpeed(const float* obs) {                 // qd block starts at 13 + ndof
    float m = 0; for (int i = 0; i < kNdof; ++i) m = std::max(m, std::fabs(obs[13 + kNdof + i]));
    return m;
}
// obs indices of the revolute (1-DOF) joints — elbows/knees/ankles, the ones with rig limits.
std::vector<int> revoluteObsIdx() {
    const auto md = physics::diff::articulationToDiffModel(physics::makeHumanoid());
    std::vector<int> idx; int off = 13;
    for (const auto& L : md.links) { if (L.dof == 1) idx.push_back(off); off += L.dof; }
    return idx;
}
} // namespace

// (1) GUARDRAIL — the two backends author the SAME state: reset obs must be bit-identical. This is
// the invariant every other comparison rests on; it must never regress.
TST_CASE(physics_env, integration, backends_reset_obs_bit_identical) {
    Environment red(humanoidCfg());        red.reset(0);
    DiffVecEnv  dif(1, humanoidCfg());     dif.reset(0);
    std::vector<float> ro(red.defaultObsDim()); red.packDefaultObs(ro);
    const auto dobs = dif.observations();
    TST_REQUIRE(red.defaultObsDim() == (int)dobs.size());
    float maxDiff = 0; for (size_t k = 0; k < ro.size(); ++k) maxDiff = std::max(maxDiff, std::fabs(ro[k] - dobs[k]));
    std::printf("backend_fidelity/reset: maxObsDiff=%.3e\n", maxDiff);
    TST_REQUIRE(maxDiff < 1e-5f);
}

// (2) CHARACTERIZATION — FINDING A (contact). Under zero action (PD holding the neutral pose) the
// reduced backend rests, but the diff backend's EXPLICIT penalty contact fires on feet that already
// penetrate at the authored pose and INJECTS energy (root rises, joints spin up). We pin: reduced is
// stable; diff gains height and stays finite (no NaN).
// FIXED (2026-07-10): with per-substep PD + SemiImplicit contact + limits/damping the diff humanoid
// now HOLDS the rest pose like reduced (no upward launch, no spin-up) instead of injecting energy.
TST_CASE(physics_env, integration, diff_rest_pose_matches_reduced) {
    Environment red(humanoidCfg());        red.reset(0);
    DiffVecEnv  dif(1, humanoidCfg());     dif.reset(0);
    std::vector<float> ro(red.defaultObsDim());
    std::vector<float> zero(red.actDim(), 0.0f);
    float redMaxDy = 0, redQd = 0, difMaxY = 0, difQd = 0; bool difFinite = true;
    for (int t = 0; t < 30; ++t) {
        red.setAction(zero); red.step();
        for (float& v : dif.actions()) v = 0.0f; dif.step();
        red.packDefaultObs(ro); const auto d = dif.observations();
        redMaxDy = std::max(redMaxDy, std::fabs(ro[1] - 0.99f));
        redQd    = std::max(redQd, maxJointSpeed(ro.data()));
        difMaxY  = std::max(difMaxY, d[1]);
        difQd    = std::max(difQd, maxJointSpeed(d.data()));
        difFinite = difFinite && std::isfinite(d[1]);
    }
    std::printf("backend_fidelity/rest: reduced(dY=%.3f qd=%.2f)  diff(maxY=%.3f qd=%.1f finite=%d)\n",
                redMaxDy, redQd, difMaxY, difQd, (int)difFinite);
    // reduced holds the pose:
    TST_REQUIRE(redMaxDy < 0.03f);
    TST_REQUIRE(redQd < 5.0f);
    // diff now rests like reduced: no upward launch, joints quiescent (was maxY~1.17, qd~500+ before
    // the per-substep-PD + SemiImplicit fix).
    TST_REQUIRE(difFinite);
    TST_REQUIRE(difMaxY < 1.05f);      // root does not launch upward (only a small settling transient)
    TST_REQUIRE(difQd < 30.0f);        // joints don't spin up
}

// (3) CHARACTERIZATION — FINDING B (joint limits). The rig gives elbows/knees/ankles hard limits
// (|angle| <= 2.5). The reduced backend enforces them; the diff DiffLink has NO limit fields and
// articulationToDiffModel drops them, so diff joints hyperextend. Drive every PD target past the
// limits and compare the largest revolute angle reached.
//   TARGET once diff gains joint limits: assert difMaxRevolute <= 2.6 as well.
TST_CASE(physics_env, integration, diff_ignores_joint_limits) {
    const auto rev = revoluteObsIdx();
    TST_REQUIRE(rev.size() == 6);                       // 2 elbows + 2 knees + 2 ankles
    Environment red(humanoidCfg());        red.reset(0);
    DiffVecEnv  dif(1, humanoidCfg());     dif.reset(0);
    std::vector<float> big(red.actDim(), 3.0f), ro(red.defaultObsDim());   // target beyond every limit
    for (int t = 0; t < 8; ++t) {
        red.setAction(big); red.step();
        for (float& v : dif.actions()) v = 3.0f; dif.step();
    }
    red.packDefaultObs(ro); const auto d = dif.observations();
    float redMax = 0, difMax = 0;
    for (int i : rev) { redMax = std::max(redMax, std::fabs(ro[i])); difMax = std::max(difMax, std::fabs(d[i])); }
    std::printf("backend_fidelity/limits: maxRevoluteAngle reduced=%.3f diff=%.3f (rig limit |.|<=2.5)\n",
                redMax, difMax);
    TST_REQUIRE(std::isfinite(difMax));
    TST_REQUIRE(redMax <= 2.6f);                        // reduced enforces the limits
    TST_REQUIRE(difMax <= 2.6f);                        // diff now enforces them too (smooth penalty)
}
