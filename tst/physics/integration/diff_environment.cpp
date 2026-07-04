//
//  diff_environment.cpp
//  engine::tst / physics / integration
//
//  Phase F3c: the differentiable environment running the REAL humanoid. Validates the
//  ArticulationDef→DiffModel converter (topology/DOF/kinematics), a finite/bounded differentiable
//  rollout, that the per-step tangent Jacobian is finite, and — the payoff — that the analytic
//  gradient of a scalar objective over a humanoid rollout (via DiffEnvironment::rolloutGradient)
//  matches central finite differences. This is the differentiable-sim boundary a downstream trainer
//  consumes.
//

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "engine/physics/diff/diff_environment.h"
#include "engine/physics/dynamics/articulation.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics::diff;

namespace {
// Double rollout from a given state/action returning a chosen joint velocity (for finite diffs).
double rolloutQd(const DiffModel& md, DiffState<double> st, const std::vector<double>& action,
                 const V3<double>& g, double h, int substeps, int nSteps, int dof) {
    for (int c = 0; c < nSteps; ++c) for (int i = 0; i < substeps; ++i) diffSubstep(md, st, action, g, h);
    return st.qd[static_cast<size_t>(dof)];
}
}

TST_CASE(physics, integration, diff_env_humanoid_converts_and_runs) {
    const DiffModel md = articulationToDiffModel(physics::makeHumanoid());
    std::printf("diff_env_convert: links=%zu ndofJoints=%d floating=%d\n", md.links.size(), md.ndofJoints, md.floating);
    TST_REQUIRE(md.links.size() == 14);
    TST_REQUIRE(md.ndofJoints == 21);          // 5 ball×3 + 6 hinge×1
    TST_REQUIRE(md.floating);                  // pelvis is the Dynamic floating root

    DiffEnvironment env(physics::makeHumanoid());
    const auto lw0 = env.links();
    std::printf("diff_env_kin: pelvisY=%.3f footLY=%.3f\n", lw0[0].pos.y, lw0[12].pos.y);
    TST_REQUIRE(std::fabs(lw0[0].pos.y - 0.99) < 1e-6);        // pelvis at authored height
    TST_REQUIRE(std::fabs(lw0[12].pos.y - 0.03) < 1e-6);       // left foot COM at authored height

    // Finite, bounded differentiable rollout under small random torques (in the air).
    std::mt19937 rng(3); std::uniform_real_distribution<double> d(-2.0, 2.0);
    std::vector<double> act(static_cast<size_t>(env.actionDim()));
    for (int c = 0; c < 30; ++c) { for (double& a : act) a = d(rng); env.setAction(act); env.step(); }
    const auto lw = env.links();
    for (const auto& w : lw) {
        TST_REQUIRE(std::isfinite(w.pos.x) && std::isfinite(w.pos.y) && std::isfinite(w.pos.z));
        TST_REQUIRE(std::fabs(w.pos.x) < 5.0 && std::fabs(w.pos.z) < 5.0 && w.pos.y > -12.0 && w.pos.y < 5.0);
    }
}

TST_CASE(physics, integration, diff_env_humanoid_gradient_matches_fd) {
    DiffEnvironment env(physics::makeHumanoid());
    const int nSteps = 5;
    std::vector<double> action(static_cast<size_t>(env.actionDim()), 0.0);
    action[0] = 1.0; action[1] = -0.3; action[2] = 0.2; action[3] = 0.5;   // waist ball xyz + shoulderL x

    // Analytic gradient of (final waist-x angular velocity) w.r.t. the first 4 action components.
    auto objective = [](const DiffState<Dual<4>>& st) { return st.qd[0]; };
    const std::vector<double> grad = env.rolloutGradient<4>(action, nSteps, objective);

    // The per-step tangent Jacobian is finite too.
    const StepJacobian J = env.jacobian();
    bool jfinite = true; for (double v : J.J) jfinite = jfinite && std::isfinite(v);
    TST_REQUIRE(jfinite);
    TST_REQUIRE(J.nState == 2 * (6 + 21) && J.nInput == J.nState + 21);   // 54 × 75

    // Finite differences on the same rollout (double), from the same reset state.
    const double h = env.substepDt(); const int nsub = env.substeps();
    const V3<double> g{ 0, -9.81, 0 };
    const double eps = 1e-6;
    double maxErr = 0;
    for (int j = 0; j < 4; ++j) {
        std::vector<double> ap = action, am = action; ap[static_cast<size_t>(j)] += eps; am[static_cast<size_t>(j)] -= eps;
        const double fd = (rolloutQd(env.model(), env.state(), ap, g, h, nsub, nSteps, 0)
                         - rolloutQd(env.model(), env.state(), am, g, h, nsub, nSteps, 0)) / (2 * eps);
        maxErr = std::max(maxErr, std::fabs(grad[static_cast<size_t>(j)] - fd));
    }
    std::printf("diff_env_grad: d(qd0)/d(act0..3) analytic=(%.5f,%.5f,%.5f,%.5f) maxErr(vs FD)=%.3e\n",
                grad[0], grad[1], grad[2], grad[3], maxErr);
    TST_REQUIRE(std::fabs(grad[0]) > 1e-3);        // waist-x torque clearly drives waist-x velocity
    TST_REQUIRE(maxErr < 1e-5);                    // analytic humanoid gradient == finite differences
}
