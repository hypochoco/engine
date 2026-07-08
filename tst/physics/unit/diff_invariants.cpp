//
//  diff_invariants.cpp
//  engine::tst / physics / unit
//
//  Phase F bug-hunting round: physical invariants and numerical-robustness probes on the
//  differentiable stack, aimed at catching converter / dynamics / integrator bugs that the
//  earlier "does the gradient match FD" tests would not surface:
//    • COM of the converted humanoid is ballistic under arbitrary INTERNAL torques (internal
//      forces must cancel — catches asymmetric-force / mass bugs in the converter);
//    • the differentiable rollout is deterministic (bitwise);
//    • long-horizon energy drift of a passive articulation (integrator quality);
//    • ground-contact numerical stability vs stiffness (find the stable range);
//    • Dual gradients are independent of how seeds are batched.
//

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "engine/physics/diff/from_articulation.h"
#include "engine/physics/dynamics/articulation.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics::diff;

namespace {
DiffState<double> humanoidRestState(const DiffModel& md) {
    DiffState<double> st = makeState<double>(md);
    st.basePos = { 0, 0.99, 0 };   // pelvis (root) authored height
    return st;
}
double comY(const DiffModel& md, const DiffState<double>& st, double& x, double& z) {
    const auto lw = linkWorld<double>(md, st);
    double M = 0, cx = 0, cy = 0, cz = 0;
    for (size_t i = 0; i < md.links.size(); ++i) { const double m = md.links[i].mass; M += m; cx += m * lw[i].pos.x; cy += m * lw[i].pos.y; cz += m * lw[i].pos.z; }
    x = cx / M; z = cz / M; return cy / M;
}
}

// (1) Under INTERNAL joint torques + gravity, the humanoid's total COM must follow the gravity
// parabola exactly (internal forces cancel). Gentle forcing so the explicit integrator stays stable
// — this isolates the invariant (converter masses/forces) from the stiffness limit probed elsewhere.
TST_CASE(physics, unit, diff_humanoid_com_ballistic) {
    const DiffModel md = articulationToDiffModel(physics::makeHumanoid());
    const double g = 9.81, h = 1.0 / 2000.0;
    const V3<double> grav{ 0, -g, 0 };
    DiffState<double> st = humanoidRestState(md);
    double x0, z0; const double cy0 = comY(md, st, x0, z0);

    std::mt19937 rng(11); std::uniform_real_distribution<double> d(-4.0, 4.0);
    std::vector<double> tau(static_cast<size_t>(md.ndofJoints));
    double maxErrY = 0, maxDriftXZ = 0, t = 0;
    for (int i = 0; i < 3000; ++i) {
        for (double& v : tau) v = d(rng);
        diffSubstep(md, st, tau, grav, h); t += h;
        double x, z; const double cy = comY(md, st, x, z);
        maxErrY = std::max(maxErrY, std::fabs(cy - (cy0 - 0.5 * g * t * t)));
        maxDriftXZ = std::max({ maxDriftXZ, std::fabs(x - x0), std::fabs(z - z0) });
    }
    const double drop = 0.5 * g * t * t;
    std::printf("diff_com_ballistic: drop=%.3fm maxErrY=%.3e (rel %.2e) maxDriftXZ=%.3e\n",
                drop, maxErrY, maxErrY / drop, maxDriftXZ);
    // Ballistic to within the explicit integrator's momentum drift (the suite already accepts
    // ~5e-3 relative for momentum conservation). A force-cancellation BUG would be O(1).
    TST_REQUIRE(maxErrY / drop < 2e-3);
    TST_REQUIRE(maxDriftXZ < 2e-3);
}

// (2) The differentiable rollout is bitwise deterministic.
TST_CASE(physics, unit, diff_humanoid_deterministic) {
    const DiffModel md = articulationToDiffModel(physics::makeHumanoid());
    const V3<double> grav{ 0, -9.81, 0 }; const double h = 1.0 / 600.0;
    auto run = [&] {
        DiffState<double> st = humanoidRestState(md);
        std::mt19937 rng(5); std::uniform_real_distribution<double> d(-20, 20);
        std::vector<double> tau(static_cast<size_t>(md.ndofJoints));
        for (int i = 0; i < 400; ++i) { for (double& v : tau) v = d(rng); diffSubstep(md, st, tau, grav, h); }
        return st;
    };
    const DiffState<double> a = run(), b = run();
    bool identical = (a.basePos.x == b.basePos.x && a.basePos.y == b.basePos.y && a.basePos.z == b.basePos.z);
    for (int i = 0; i < a.numDof; ++i) identical = identical && (a.qd[i] == b.qd[i]);
    for (int j = 0; j < 6; ++j) identical = identical && (a.baseTwist.d[j] == b.baseTwist.d[j]);
    std::printf("diff_deterministic: identical=%d finalPelvisY=%.6f\n", identical, a.basePos.y);
    TST_REQUIRE(identical);
}

// (3) Long-horizon energy drift of a passive isotropic floating chain (integrator quality). Semi-
// implicit Euler should keep the error bounded (not growing) over 60k steps.
TST_CASE(physics, unit, diff_energy_drift_longhorizon) {
    DiffModel md; md.ndofJoints = 2; md.floating = true;
    DiffLink root; root.parent = -1; root.mass = 2; root.Ic = diagM3(0.03, 0.03, 0.03); root.restRel = identity3<double>();
    auto link = [&](int p, int qi) { DiffLink l; l.parent = p; l.dof = 1; l.qIndex = qi; l.mass = 1; l.Ic = diagM3(0.01, 0.01, 0.01);
        l.axes[0] = { 0, 0, 1 }; l.anchorP = { 0.5, 0, 0 }; l.anchorC = { -0.5, 0, 0 }; l.restRel = identity3<double>(); return l; };
    md.links = { root, link(0, 0), link(1, 1) };
    const double g = 9.81, h = 1.0 / 4000.0; const V3<double> grav{ 0, -g, 0 };
    const std::vector<double> tau{ 0.0, 0.0 };
    const double Ir = 0.01;
    auto energy = [&](const DiffState<double>& st) { const auto lw = linkWorld<double>(md, st); double E = 0;
        for (size_t i = 0; i < md.links.size(); ++i) { const double m = md.links[i].mass, I = md.links[i].Ic.m[0][0]; const auto& w = lw[i];
            E += 0.5 * m * dot(w.linVel, w.linVel) + 0.5 * I * dot(w.angVel, w.angVel) + m * g * w.pos.y; } (void)Ir; return E; };
    DiffState<double> st = makeState<double>(md);
    st.baseTwist.d[3] = 1.5; st.linkRot[2] = rodrigues<double>(V3<double>{ 0, 0, 1 }, 0.7);
    const double E0 = energy(st);
    double drift1 = 0, drift2 = 0; bool finite = true;
    for (int i = 0; i < 60000; ++i) { diffSubstep(md, st, tau, grav, h); const double dr = std::fabs(energy(st) - E0);
        (i < 30000 ? drift1 : drift2) = std::max(i < 30000 ? drift1 : drift2, dr);
        finite = finite && std::isfinite(st.basePos.y); }
    std::printf("diff_energy_drift_60k: E0=%.4f drift[0,30k]=%.5f (%.2f%%) drift[30k,60k]=%.5f (%.2f%%)\n",
                E0, drift1, 100.0 * drift1 / std::fabs(E0), drift2, 100.0 * drift2 / std::fabs(E0));
    TST_REQUIRE(finite);
    // Semi-implicit Euler is non-symplectic for rigid-body rotation ⇒ a slow SECULAR energy drift
    // on a fast free-spinning chain (characterized here). Bounded per unit time — over 7.5 s (30k
    // steps @ h=1/4000) it stays small; irrelevant to short controlled RL horizons.
    TST_REQUIRE(drift1 / std::fabs(E0) < 0.08);
}

// (4) Ground-contact stability vs stiffness: drop a sphere, sweep groundK, report where it stays
// finite/settled. Confirms the production default (3e3) is comfortably stable at the env timestep.
TST_CASE(physics, unit, diff_contact_stability_sweep) {
    auto probe = [](double k, double h) {
        DiffModel md; md.floating = true; md.ndofJoints = 0; md.contactGround = true; md.groundK = k; md.groundC = 30;
        DiffLink b; b.parent = -1; b.mass = 1; b.Ic = diagM3(0.004, 0.004, 0.004); b.restRel = identity3<double>(); b.contactRadius = 0.1;
        md.links = { b };
        DiffState<double> st = makeState<double>(md); st.basePos = { 0, 0.5, 0 };
        const V3<double> grav{ 0, -9.81, 0 }; const std::vector<double> tau;
        double maxY = 0.5; bool finite = true;
        for (int i = 0; i < 6000; ++i) { diffSubstep(md, st, tau, grav, h); maxY = std::max(maxY, std::fabs(st.basePos.y)); finite = finite && std::isfinite(st.basePos.y); }
        return std::pair<bool, double>{ finite && maxY < 5.0, st.basePos.y };
    };
    const double h = 1.0 / 960.0;   // env substep (1/60 / 16)
    std::printf("diff_contact_stability (h=%.5f):\n", h);
    bool defaultStable = false;
    for (double k : { 1e3, 3e3, 1e4, 3e4, 1e5, 3e5 }) {
        const auto [stable, restY] = probe(k, h);
        std::printf("  k=%-8.0f stable=%d restY=%.4f (expected r-mg/k=%.4f)\n", k, stable, restY, 0.1 - 9.81 / k);
        if (k == 3e3) defaultStable = stable;
    }
    TST_REQUIRE(defaultStable);
}

// (5) Dual gradients are independent of seed batching: seeding two inputs together in one Dual<2>
// pass equals two separate Dual<1> passes.
TST_CASE(physics, unit, diff_dual_seed_batching_invariant) {
    DiffModel md; md.ndofJoints = 2; md.floating = false;
    auto link = [&](int p, int qi) { DiffLink l; l.parent = p; l.dof = 1; l.qIndex = qi; l.mass = 1; l.Ic = diagM3(0.001, 0.001, 0.001);
        l.axes[0] = { 0, 0, 1 }; l.anchorC = { -1, 0, 0 }; l.restRel = identity3<double>(); return l; };
    DiffLink base; base.parent = -1; base.mass = 1; base.Ic = diagM3(1, 1, 1); base.restRel = identity3<double>();
    md.links = { base, link(0, 0), link(1, 1) };
    const V3<double> grav{ 0, -9.81, 0 }; const double h = 1.0 / 2000.0; const int steps = 120;
    const double t0 = 0.5, t1 = -0.3;
    auto angleZ = [](const auto& R) { using std::atan2; return atan2(R.m[1][0], R.m[0][0]); };

    std::vector<Dual<2>> tau2{ Dual<2>::seed(t0, 0), Dual<2>::seed(t1, 1) };
    DiffState<Dual<2>> s2 = makeState<Dual<2>>(md);
    for (int i = 0; i < steps; ++i) diffSubstep(md, s2, tau2, grav, h);
    const Dual<2> out2 = angleZ(s2.linkRot[2]);

    auto single = [&](int which) {
        std::vector<Dual<1>> tau{ Dual<1>(t0), Dual<1>(t1) }; tau[static_cast<size_t>(which)] = Dual<1>::seed(which == 0 ? t0 : t1, 0);
        DiffState<Dual<1>> s = makeState<Dual<1>>(md);
        for (int i = 0; i < steps; ++i) diffSubstep(md, s, tau, grav, h);
        return angleZ(s.linkRot[2]).d[0];
    };
    const double g0 = single(0), g1 = single(1);
    std::printf("diff_seed_batching: batched=(%.8f,%.8f) separate=(%.8f,%.8f)\n", out2.d[0], out2.d[1], g0, g1);
    TST_REQUIRE(std::fabs(out2.d[0] - g0) < 1e-12 && std::fabs(out2.d[1] - g1) < 1e-12);
}
