//
//  diff_validation.cpp
//  engine::tst / physics / integration
//
//  Phase F bug-hunting round (cross-cutting): validate the differentiable humanoid against the
//  PRODUCTION reduced backend and probe the fragile contact paths:
//    • converter fidelity — a passive (zero-torque) humanoid falls identically on the diff ABA and
//      the reduced Featherstone backend (catches wrong inertia/anchor/axis conventions);
//    • gradient THROUGH contact on the real humanoid == finite differences (contact is where
//      differentiable sim is most fragile);
//    • the per-step Jacobian at a contact state is finite;
//    • a long dropped-humanoid contact rollout stays finite/bounded;
//    • first-order gradient magnitude vs horizon through contact (the pathology the hybrid targets).
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/diff_environment.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics::diff;

namespace {
// Double contact rollout returning the final pelvis height (for finite differences).
double rolloutPelvisY(const DiffModel& md, DiffState<double> st, const std::vector<double>& action,
                      const V3<double>& g, double h, int substeps, int nSteps) {
    for (int c = 0; c < nSteps; ++c) for (int i = 0; i < substeps; ++i) diffSubstep(md, st, action, g, h);
    return st.basePos.y;
}
}

// (1) Converter fidelity: a passive humanoid must fall the SAME way on the differentiable ABA and
// the production reduced backend (no torques, no contact, no damping).
TST_CASE(physics, integration, diff_matches_reduced_backend) {
    const physics::ArticulationDef def = physics::makeHumanoid();
    const double controlDt = 1.0 / 60.0; const int S = 8; const double h = controlDt / S;
    const int K = 10;   // 0.167 s of passive sag

    // reduced backend
    physics::WorldDef wd; wd.gravity = physics::Vec3(0, -9.81f, 0); wd.substeps = S;
    wd.linearDamping = 0; wd.angularDamping = 0;
    auto world = physics::createPhysicsWorld(physics::Backend::Reduced, wd);
    const physics::Articulation art = physics::buildArticulation(*world, def);
    for (int c = 0; c < K; ++c) world->step(static_cast<float>(controlDt));

    // differentiable
    const DiffModel md = articulationToDiffModel(def);
    DiffState<double> st = makeState<double>(md); st.basePos = { 0, 0.99, 0 };
    const V3<double> grav{ 0, -9.81, 0 }; const std::vector<double> tau(static_cast<size_t>(md.ndofJoints), 0.0);
    for (int c = 0; c < K; ++c) for (int i = 0; i < S; ++i) diffSubstep(md, st, tau, grav, h);
    const auto lw = linkWorld<double>(md, st);

    double maxDiff = 0; int worst = -1;
    for (size_t i = 0; i < def.bodies.size(); ++i) {
        const glm::vec3 rp = world->pose(art.bodies[i]).position;
        const double dx = rp.x - lw[i].pos.x, dy = rp.y - lw[i].pos.y, dz = rp.z - lw[i].pos.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist > maxDiff) { maxDiff = dist; worst = static_cast<int>(i); }
    }
    const glm::vec3 rootRed = world->pose(art.bodies[0]).position;
    std::printf("diff_vs_reduced: pelvisY reduced=%.5f diff=%.5f | max body pos diff=%.3e m (body %d)\n",
                rootRed.y, lw[0].pos.y, maxDiff, worst);
    TST_REQUIRE(maxDiff < 5e-3);   // trajectories agree (float vs double + any solver nuance)
}

// (2) Gradient through CONTACT on the real humanoid == central finite differences.
TST_CASE(physics, integration, diff_humanoid_contact_gradient_matches_fd) {
    DiffEnvironment env(physics::makeHumanoid(), /*footContactRadius=*/0.03);   // feet touch the plane at reset
    const int nSteps = 4;
    std::vector<double> action(static_cast<size_t>(env.actionDim()), 0.0);
    action[0] = 6.0; action[1] = -4.0; action[7] = 5.0; action[8] = -5.0;   // waist + hips

    auto objective = [](const DiffState<Dual<4>>& st) { return st.basePos.y; };   // final pelvis height
    const std::vector<double> grad = env.rolloutGradient<4>(action, nSteps, objective);

    const StepJacobian J = env.jacobian();
    bool jfinite = true; for (double v : J.J) jfinite = jfinite && std::isfinite(v);

    const double h = env.substepDt(); const int nsub = env.substeps(); const V3<double> g{ 0, -9.81, 0 };
    const double eps = 1e-6; double maxErr = 0;
    for (int j = 0; j < 4; ++j) {
        std::vector<double> ap = action, am = action; ap[static_cast<size_t>(j)] += eps; am[static_cast<size_t>(j)] -= eps;
        const double fd = (rolloutPelvisY(env.model(), env.state(), ap, g, h, nsub, nSteps)
                         - rolloutPelvisY(env.model(), env.state(), am, g, h, nsub, nSteps)) / (2 * eps);
        maxErr = std::max(maxErr, std::fabs(grad[static_cast<size_t>(j)] - fd));
    }
    std::printf("diff_contact_grad: grad=(%.3e,%.3e,%.3e,%.3e) maxErr(vs FD)=%.3e jacFinite=%d\n",
                grad[0], grad[1], grad[2], grad[3], maxErr, jfinite);
    TST_REQUIRE(jfinite);
    TST_REQUIRE(maxErr < 1e-6);   // analytic contact gradient == FD
}

// (3) CHARACTERIZATION: a passive (uncontrolled) humanoid dropped onto foot contact — how the
// forward stability depends on the substep count (timestep). Determines whether the humanoid-
// contact instability is a fixable timestep/stiffness issue.
TST_CASE(physics, integration, diff_humanoid_contact_rollout_stable) {
    std::printf("diff_contact_rollout (passive drop, 5 s @ 1/60):\n");
    int bestSteps = 0; int finestSurvived = 0;
    for (int substeps : { 16, 32, 64, 128, 256 }) {
        DiffEnvironment env(physics::makeHumanoid(), 0.03, { 0, -9.81, 0 }, 1.0 / 60.0, substeps);
        std::vector<double> act(static_cast<size_t>(env.actionDim()), 0.0);
        int survived = 0; double maxAbsY = 0; bool finite = true;
        for (int c = 0; c < 300 && finite; ++c) {
            env.setAction(act); env.step();
            for (const auto& w : env.links()) { finite = finite && std::isfinite(w.pos.y); maxAbsY = std::max(maxAbsY, std::fabs(w.pos.y)); }
            if (finite && maxAbsY < 10.0) survived = c + 1;
        }
        std::printf("  substeps=%-4d survived=%d/300 control steps  maxAbsY=%.2f\n", substeps, survived, maxAbsY);
        bestSteps = std::max(bestSteps, survived);
        if (substeps == 256) finestSurvived = survived;
    }
    // FINDING: the default (substeps=16, h≈1 ms) is below the stability threshold for stiff foot
    // contact through the articulated legs; substeps≥64 is stable. So it is TIMESTEP-fixable.
    std::printf("  -> best survival = %d control steps (finest=256 -> %d)\n", bestSteps, finestSurvived);
    TST_REQUIRE(finestSurvived == 300);
}

// (4) MEASUREMENT: first-order gradient magnitude vs horizon through contact — quantifies the
// exploding-gradient pathology that motivates the α-order hybrid. Reported, not asserted (beyond
// finiteness).
TST_CASE(physics, integration, diff_contact_gradient_vs_horizon) {
    DiffEnvironment env(physics::makeHumanoid(), 0.03);
    std::vector<double> action(static_cast<size_t>(env.actionDim()), 0.0);
    action[7] = 8.0; action[8] = -8.0;   // hips
    auto objective = [](const DiffState<Dual<2>>& st) { return st.basePos.y; };
    std::printf("diff_grad_vs_horizon (||d(pelvisY)/d(hipTorque)||):\n");
    bool shortFinite = true;
    for (int H : { 1, 2, 4, 8, 16, 32, 64 }) {
        const auto g = env.rolloutGradient<2>(action, H, objective);
        const double norm = std::sqrt(g[0] * g[0] + g[1] * g[1]);
        if (H <= 8) shortFinite = shortFinite && std::isfinite(norm);
        std::printf("  H=%-3d |grad|=%.4e%s\n", H, norm, std::isfinite(norm) ? "" : "  <-- diverged");
    }
    TST_REQUIRE(shortFinite);   // short horizons (the SHAC regime) are well-behaved
}

// (5) With contact geometry on EVERY body (not just feet), a passive collapsing humanoid must pile
// ON the plane, not phase THROUGH it. Guards against the sink-through the diff_humanoid visual first
// exposed (feet-only contact + uncontrolled collapse). minBodyY staying above the plane is the check
// that "finite but fell through the floor" no longer passes as "stable".
TST_CASE(physics, integration, diff_humanoid_rests_on_ground) {
    const physics::ArticulationDef def = physics::makeHumanoid();
    DiffModel md = articulationToDiffModel(def, 0.03);
    md.contactGround = true;
    for (size_t i = 0; i < md.links.size(); ++i) {
        const physics::ColliderDesc& col = def.bodies[i].collider;
        double r = 0.05;
        if (col.type == physics::ColliderDesc::Type::Box) r = std::min({ col.box.halfExtents.x, col.box.halfExtents.y, col.box.halfExtents.z });
        else if (col.type == physics::ColliderDesc::Type::Capsule) r = col.capsule.radius;
        md.links[i].contactRadius = r;
    }
    DiffState<double> st = makeState<double>(md); st.basePos = { 0, 0.99, 0 };
    const V3<double> grav{ 0, -9.81, 0 }; const int substeps = 64; const double h = (1.0 / 60.0) / substeps;
    const std::vector<double> tau(static_cast<size_t>(md.ndofJoints), 0.0);

    double minSettledY = 1e9; bool finite = true;
    for (int c = 0; c < 300; ++c) {
        for (int i = 0; i < substeps; ++i) diffSubstep(md, st, tau, grav, h);
        const auto lw = linkWorld<double>(md, st);
        for (const auto& w : lw) finite = finite && std::isfinite(w.pos.y);
        if (c >= 240) for (const auto& w : lw) minSettledY = std::min(minSettledY, (double)w.pos.y);   // last ~1 s
    }
    const double pelvisY = linkWorld<double>(md, st)[0].pos.y;
    std::printf("diff_rests_on_ground: finite=%d settled pelvisY=%.4f minBodyY(last 1s)=%.4f\n", finite, pelvisY, minSettledY);
    TST_REQUIRE(finite);
    TST_REQUIRE(minSettledY > -0.1);     // rests ON the plane — no phase-through
    TST_REQUIRE(pelvisY < 0.6);          // and it did collapse (uncontrolled), not magically stand
}
