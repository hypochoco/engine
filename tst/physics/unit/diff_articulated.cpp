//
//  diff_articulated.cpp
//  engine::tst / physics / unit
//
//  Phase F1c/F1d: validate the Scalar-generic ABA (include/engine/physics/diff/articulated.h) —
//  the differentiable multi-DOF dynamics for fixed/floating base and revolute/ball/fixed joints.
//  Physical checks (period, energy, momentum) confirm the dynamics; gradient checks (forward-mode
//  `Dual` vs central finite differences) confirm exact differentiability. Quaternion-free.
//

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/articulated.h"
#include "harness/harness.h"

using namespace engine::physics::diff;

namespace {

template <class S> S angleZ(const M3<S>& R) { using std::atan2; return atan2(R.m[1][0], R.m[0][0]); }

template <class S>
DiffState<S> runChain(const DiffModel& md, DiffState<S> st, const std::vector<S>& tau,
                      const V3<double>& g, double h, int steps) {
    for (int i = 0; i < steps; ++i) diffSubstep(md, st, tau, g, h);
    return st;
}

DiffLink baseLink() { DiffLink b; b.parent = -1; b.mass = 1; b.Ic = diagM3(1, 1, 1); b.restRel = identity3<double>(); return b; }

// fixed base + one revolute bob; restRel = Rot(z,−90°) ⇒ q=0 is straight-down equilibrium.
DiffModel singlePendulum(double m, double L, double Ic) {
    DiffModel md; md.ndofJoints = 1; md.floating = false;
    DiffLink bob; bob.parent = 0; bob.dof = 1; bob.qIndex = 0; bob.mass = m; bob.Ic = diagM3(Ic, Ic, Ic);
    bob.axes[0] = { 0, 0, 1 }; bob.anchorC = { -L, 0, 0 };
    bob.restRel = rodrigues<double>(V3<double>{ 0, 0, 1 }, -M_PI / 2);
    md.links = { baseLink(), bob };
    return md;
}
// fixed base + two revolute links built horizontal (+x).
DiffModel doublePendulum(double m, double L, double Ic) {
    DiffModel md; md.ndofJoints = 2; md.floating = false;
    auto link = [&](int parent, int qi) { DiffLink l; l.parent = parent; l.dof = 1; l.qIndex = qi; l.mass = m;
        l.Ic = diagM3(Ic, Ic, Ic); l.axes[0] = { 0, 0, 1 }; l.anchorC = { -L, 0, 0 }; l.restRel = identity3<double>(); return l; };
    md.links = { baseLink(), link(0, 0), link(1, 1) };
    return md;
}
// fixed base + one BALL bob built horizontal (+x).
DiffModel ballPendulum(double m, double L, double Ic) {
    DiffModel md; md.ndofJoints = 3; md.floating = false;
    DiffLink bob; bob.parent = 0; bob.dof = 3; bob.qIndex = 0; bob.mass = m; bob.Ic = diagM3(Ic, Ic, Ic);
    bob.anchorC = { -L, 0, 0 }; bob.restRel = identity3<double>();   // axes default x,y,z
    md.links = { baseLink(), bob };
    return md;
}
// FLOATING sphere root + two revolute links (isotropic inertias ⇒ rotation-invariant angular momentum).
DiffModel floatingChain() {
    DiffModel md; md.ndofJoints = 2; md.floating = true;
    DiffLink root; root.parent = -1; root.mass = 2; root.Ic = diagM3(0.032, 0.032, 0.032); root.restRel = identity3<double>();
    auto link = [&](int parent, int qi, double m, double I) { DiffLink l; l.parent = parent; l.dof = 1; l.qIndex = qi;
        l.mass = m; l.Ic = diagM3(I, I, I); l.axes[0] = { 0, 0, 1 }; l.anchorP = { 0.5, 0, 0 }; l.anchorC = { -0.5, 0, 0 };
        l.restRel = identity3<double>(); return l; };
    md.links = { root, link(0, 0, 1.0, 0.009), link(1, 1, 1.0, 0.009) };
    return md;
}

} // namespace

// --- (1) single revolute pendulum period == analytic -------------------------------------------
TST_CASE(physics, unit, diff_aba_single_pendulum_period) {
    const double m = 1.0, L = 1.0, Ic = 0.001, g = 9.81, h = 1.0 / 4000.0;
    const DiffModel md = singlePendulum(m, L, Ic);
    const double expected = 2.0 * M_PI * std::sqrt((m * L * L + Ic) / (m * g * L));
    const V3<double> grav{ 0, -g, 0 };
    const std::vector<double> tau{ 0.0 };

    DiffState<double> st = makeState<double>(md);
    st.linkRot[1] = rodrigues<double>(V3<double>{ 0, 0, 1 }, 0.05);       // release 0.05 rad off straight-down
    double t = 0, prev = 0.05, tDown = -1, tUp = -1, minQ = 0.05;
    for (int i = 0; i < 20000; ++i) {
        diffSubstep(md, st, tau, grav, h); t += h;
        const double q = angleZ(st.linkRot[1]);
        minQ = std::min(minQ, q);
        if (tDown < 0 && prev > 0 && q <= 0) tDown = t;
        else if (tDown > 0 && tUp < 0 && prev < 0 && q >= 0) tUp = t;
        prev = q;
    }
    const double measured = 2.0 * (tUp - tDown);
    std::printf("diff_aba_pendulum: measured=%.4f expected=%.4f minQ=%.4f\n", measured, expected, minQ);
    TST_REQUIRE(tDown > 0 && tUp > 0);
    TST_REQUIRE(std::fabs(measured - expected) / expected < 0.02);
    TST_REQUIRE(minQ < -0.9 * 0.05);
}

// --- (2) double pendulum conserves energy ------------------------------------------------------
TST_CASE(physics, unit, diff_aba_double_pendulum_energy) {
    const double m = 1.0, Ic = 0.001, g = 9.81, h = 1.0 / 4000.0;
    const DiffModel md = doublePendulum(m, 1.0, Ic);
    const V3<double> grav{ 0, -g, 0 };
    const std::vector<double> tau{ 0.0, 0.0 };
    auto totalE = [&](const DiffState<double>& st) {
        const auto lw = linkWorld<double>(md, st); double E = 0;
        for (int i = 1; i <= 2; ++i) { const auto& w = lw[static_cast<size_t>(i)];
            E += 0.5 * m * dot(w.linVel, w.linVel) + 0.5 * Ic * dot(w.angVel, w.angVel) + m * g * w.pos.y; }
        return E;
    };
    auto pe = [&](const DiffState<double>& st) { const auto lw = linkWorld<double>(md, st);
        return m * g * (lw[1].pos.y + lw[2].pos.y); };

    DiffState<double> st = makeState<double>(md);
    const double E0 = totalE(st);
    double maxDrift = 0, peMin = pe(st), peMax = peMin;
    for (int i = 0; i < 8000; ++i) { diffSubstep(md, st, tau, grav, h);
        maxDrift = std::max(maxDrift, std::fabs(totalE(st) - E0)); const double p = pe(st); peMin = std::min(peMin, p); peMax = std::max(peMax, p); }
    const double scale = std::max(1.0, peMax - peMin);
    std::printf("diff_aba_double_energy: maxDrift=%.5f scale=%.4f (%.2f%%)\n", maxDrift, scale, 100.0 * maxDrift / scale);
    TST_REQUIRE(maxDrift / scale < 0.02);
}

// --- (3) revolute-chain gradient via Dual == central FD ----------------------------------------
TST_CASE(physics, unit, diff_aba_revolute_gradient_matches_fd) {
    const double m = 1.0, Ic = 0.001, g = 9.81, h = 1.0 / 2000.0;
    const int steps = 200;
    const DiffModel md = doublePendulum(m, 1.0, Ic);
    const V3<double> grav{ 0, -g, 0 };
    const double a0 = 0.3, t0 = 0.5;

    DiffState<Dual<2>> st = makeState<Dual<2>>(md);
    st.linkRot[1] = rodrigues<Dual<2>>(V3<Dual<2>>{ Dual<2>(0), Dual<2>(0), Dual<2>(1) }, Dual<2>::seed(a0, 0));  // seed initial angle
    const std::vector<Dual<2>> tau{ Dual<2>::seed(t0, 1), Dual<2>(0.0) };                                          // seed joint-0 torque
    const Dual<2> qf = angleZ(runChain(md, st, tau, grav, h, steps).linkRot[2]);
    const double dA = qf.d[0], dT = qf.d[1];

    auto finalQ1 = [&](double a, double t) {
        DiffState<double> s = makeState<double>(md);
        s.linkRot[1] = rodrigues<double>(V3<double>{ 0, 0, 1 }, a);
        return angleZ(runChain(md, s, std::vector<double>{ t, 0.0 }, grav, h, steps).linkRot[2]);
    };
    const double eps = 1e-6;
    const double fdA = (finalQ1(a0 + eps, t0) - finalQ1(a0 - eps, t0)) / (2 * eps);
    const double fdT = (finalQ1(a0, t0 + eps) - finalQ1(a0, t0 - eps)) / (2 * eps);
    std::printf("diff_aba_rev_grad: d/da0 AD=%.8f FD=%.8f | d/dtau0 AD=%.8f FD=%.8f\n", dA, fdA, dT, fdT);
    TST_REQUIRE(std::fabs(dA - fdA) < 1e-5);
    TST_REQUIRE(std::fabs(dT - fdT) < 1e-5);
    TST_REQUIRE(std::fabs(dA) > 1e-3 && std::fabs(dT) > 1e-4);
}

// --- (4) BALL (3-DOF) pendulum conserves energy + swings down ----------------------------------
TST_CASE(physics, unit, diff_aba_ball_pendulum_energy) {
    const double m = 1.0, L = 1.0, Ic = 0.001, g = 9.81, h = 1.0 / 4000.0;
    const DiffModel md = ballPendulum(m, L, Ic);
    const V3<double> grav{ 0, -g, 0 };
    const std::vector<double> tau{ 0.0, 0.0, 0.0 };
    auto totalE = [&](const DiffState<double>& st) { const auto lw = linkWorld<double>(md, st); const auto& w = lw[1];
        return 0.5 * m * dot(w.linVel, w.linVel) + 0.5 * Ic * dot(w.angVel, w.angVel) + m * g * w.pos.y; };

    DiffState<double> st = makeState<double>(md);
    const double E0 = totalE(st);
    double maxDrift = 0, minY = 0, peMin = 0, peMax = 0;
    for (int i = 0; i < 8000; ++i) { diffSubstep(md, st, tau, grav, h);
        const auto lw = linkWorld<double>(md, st); const double y = lw[1].pos.y;
        maxDrift = std::max(maxDrift, std::fabs(totalE(st) - E0)); minY = std::min(minY, y);
        peMin = std::min(peMin, m * g * y); peMax = std::max(peMax, m * g * y); }
    const double scale = std::max(1.0, peMax - peMin);
    std::printf("diff_aba_ball_energy: maxDrift=%.5f scale=%.4f (%.2f%%) minY=%.4f\n", maxDrift, scale, 100.0 * maxDrift / scale, minY);
    TST_REQUIRE(minY < -0.9 * L);                 // ball joint swings down through the bottom
    TST_REQUIRE(maxDrift / scale < 0.02);
}

// --- (5) ball gradient via Dual == central FD --------------------------------------------------
TST_CASE(physics, unit, diff_aba_ball_gradient_matches_fd) {
    const double m = 1.0, L = 1.0, Ic = 0.001, g = 9.81, h = 1.0 / 2000.0;
    const int steps = 150;
    const DiffModel md = ballPendulum(m, L, Ic);
    const V3<double> grav{ 0, -g, 0 };
    const double tz = 0.6, tx = 0.3;              // seed z- and x-axis ball torques

    DiffState<Dual<2>> st = makeState<Dual<2>>(md);
    std::vector<Dual<2>> tau{ Dual<2>::seed(tx, 1), Dual<2>(0.0), Dual<2>::seed(tz, 0) };   // [x,y,z]
    const auto lw = linkWorld<Dual<2>>(md, runChain(md, st, tau, grav, h, steps));
    const Dual<2> yEnd = lw[1].pos.y;
    const double dTz = yEnd.d[0], dTx = yEnd.d[1];

    auto finalY = [&](double x, double z) {
        DiffState<double> s = makeState<double>(md);
        const auto w = linkWorld<double>(md, runChain(md, s, std::vector<double>{ x, 0.0, z }, grav, h, steps));
        return w[1].pos.y;
    };
    const double eps = 1e-6;
    const double fdTz = (finalY(tx, tz + eps) - finalY(tx, tz - eps)) / (2 * eps);
    const double fdTx = (finalY(tx + eps, tz) - finalY(tx - eps, tz)) / (2 * eps);
    std::printf("diff_aba_ball_grad: d/dTz AD=%.8f FD=%.8f | d/dTx AD=%.8f FD=%.8f\n", dTz, fdTz, dTx, fdTx);
    TST_REQUIRE(std::fabs(dTz - fdTz) < 1e-5);
    TST_REQUIRE(std::fabs(dTx - fdTx) < 1e-5);
}

// --- (6) FLOATING base free chain conserves linear + angular momentum --------------------------
TST_CASE(physics, unit, diff_aba_floating_momentum) {
    const DiffModel md = floatingChain();
    const V3<double> grav{ 0, 0, 0 };                        // free space
    const std::vector<double> tau{ 0.0, 0.0 };
    auto momentum = [&](const DiffState<double>& st, V3<double>& P, V3<double>& Lang) {
        const auto lw = linkWorld<double>(md, st); P = zeroV3<double>(); Lang = zeroV3<double>();
        for (int i = 0; i < 3; ++i) { const double mi = md.links[static_cast<size_t>(i)].mass, Ici = md.links[static_cast<size_t>(i)].Ic.m[0][0];
            const auto& w = lw[static_cast<size_t>(i)];
            P = P + mi * w.linVel;
            Lang = Lang + cross(w.pos, mi * w.linVel) + Ici * w.angVel; }
    };
    DiffState<double> st = makeState<double>(md);
    st.baseTwist.d[2] = 0.8; st.baseTwist.d[3] = 0.5; st.baseTwist.d[4] = 0.3; st.baseTwist.d[5] = 0.1;  // ω_z + v
    st.linkRot[2] = rodrigues<double>(V3<double>{ 0, 0, 1 }, 0.6);   // bend link2 so the spin articulates the joints

    V3<double> P0, L0; momentum(st, P0, L0);
    const double h = 1.0 / 4000.0;
    double maxdP = 0, maxdL = 0, jointMove = 0;
    for (int i = 0; i < 4000; ++i) { diffSubstep(md, st, tau, grav, h);
        V3<double> P, Lc; momentum(st, P, Lc);
        maxdP = std::max(maxdP, std::sqrt(dot(P - P0, P - P0)));
        maxdL = std::max(maxdL, std::sqrt(dot(Lc - L0, Lc - L0)));
        jointMove = std::max({ jointMove, std::fabs(angleZ(st.linkRot[1])), std::fabs(angleZ(st.linkRot[2]) - 0.6) }); }
    const double nP0 = std::sqrt(dot(P0, P0)), nL0 = std::sqrt(dot(L0, L0));
    std::printf("diff_aba_floating: |P0|=%.4f |L0|=%.4f maxdP=%.3e maxdL=%.3e jointMove=%.3f\n", nP0, nL0, maxdP, maxdL, jointMove);
    TST_REQUIRE(nP0 > 0.1 && jointMove > 0.02);              // real motion + the chain articulates
    TST_REQUIRE(maxdP / nP0 < 5e-3);                          // linear momentum conserved
    TST_REQUIRE(maxdL / std::max(0.1, nL0) < 5e-3);           // angular momentum conserved
}

// --- (7) floating-base gradient via Dual == central FD -----------------------------------------
TST_CASE(physics, unit, diff_aba_floating_gradient_matches_fd) {
    const DiffModel md = floatingChain();
    const V3<double> grav{ 0, 0, 0 };
    const int steps = 200; const double h = 1.0 / 2000.0;
    const double wz = 0.8, t0 = 0.4;                          // seed initial base spin ω_z + joint-0 torque

    DiffState<Dual<2>> st = makeState<Dual<2>>(md);
    st.baseTwist.d[2] = Dual<2>::seed(wz, 0);
    std::vector<Dual<2>> tau{ Dual<2>::seed(t0, 1), Dual<2>(0.0) };
    const auto res = runChain(md, st, tau, grav, h, steps);
    const Dual<2> px = res.basePos.x;                          // final base x-position
    const double dWz = px.d[0], dT0 = px.d[1];

    auto finalBaseX = [&](double w, double t) {
        DiffState<double> s = makeState<double>(md); s.baseTwist.d[2] = w;
        return runChain(md, s, std::vector<double>{ t, 0.0 }, grav, h, steps).basePos.x;
    };
    const double eps = 1e-6;
    const double fdWz = (finalBaseX(wz + eps, t0) - finalBaseX(wz - eps, t0)) / (2 * eps);
    const double fdT0 = (finalBaseX(wz, t0 + eps) - finalBaseX(wz, t0 - eps)) / (2 * eps);
    std::printf("diff_aba_float_grad: d/dwz AD=%.8f FD=%.8f | d/dtau0 AD=%.8f FD=%.8f\n", dWz, fdWz, dT0, fdT0);
    TST_REQUIRE(std::fabs(dWz - fdWz) < 1e-5);
    TST_REQUIRE(std::fabs(dT0 - fdT0) < 1e-5);
}
