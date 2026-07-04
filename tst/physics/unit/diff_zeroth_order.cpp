//
//  diff_zeroth_order.cpp
//  engine::tst / physics / unit
//
//  Phase Fd: validate the zeroth-order (randomized-smoothing) gradient estimator. (1) On a quadratic
//  it is unbiased for the analytic gradient Ax+b and its accuracy improves with the sample count.
//  (2) On a smooth differentiable-dynamics objective (the pendulum) the zeroth-order estimate agrees
//  with the analytic forward-mode `Dual` gradient — establishing that the two gradient sources the
//  α-order hybrid (F3) blends coincide on smooth problems.
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/articulated.h"
#include "engine/physics/diff/zeroth_order.h"
#include "harness/harness.h"

using namespace engine::physics::diff;

namespace {

template <class S> S angleZ(const M3<S>& R) { using std::atan2; return atan2(R.m[1][0], R.m[0][0]); }

// fixed base + one revolute bob (built at q0), driven by a scalar hip torque; returns final angle.
DiffModel pendModel() {
    DiffModel md; md.ndofJoints = 1; md.floating = false;
    DiffLink base; base.parent = -1; base.mass = 1; base.Ic = diagM3(1, 1, 1); base.restRel = identity3<double>();
    DiffLink bob; bob.parent = 0; bob.dof = 1; bob.qIndex = 0; bob.mass = 1; bob.Ic = diagM3(0.001, 0.001, 0.001);
    bob.axes[0] = { 0, 0, 1 }; bob.anchorC = { -1, 0, 0 }; bob.restRel = rodrigues<double>(V3<double>{ 0, 0, 1 }, -M_PI / 2);
    md.links = { base, bob };
    return md;
}
template <class S>
S pendFinalAngle(const DiffModel& md, double q0, S tau, const V3<double>& g, double h, int steps) {
    DiffState<S> st = makeState<S>(md);
    st.linkRot[1] = rodrigues<S>(V3<S>{ S(0), S(0), S(1) }, S(q0));
    const std::vector<S> t{ tau };
    for (int i = 0; i < steps; ++i) diffSubstep(md, st, t, g, h);
    return angleZ(st.linkRot[1]);
}

} // namespace

// --- (1) unbiased on a quadratic; accuracy improves with N -------------------------------------
TST_CASE(physics, unit, diff_zeroth_order_quadratic) {
    // f(x) = ½ xᵀA x + bᵀx  ⇒  ∇f = A x + b (smoothing is unbiased for a quadratic).
    const double A[3][3] = { { 2.0, 0.5, 0.0 }, { 0.5, 3.0, 0.2 }, { 0.0, 0.2, 1.0 } };
    const std::vector<double> b = { 1.0, -2.0, 0.5 }, x = { 0.3, -0.1, 0.4 };
    auto f = [&](const std::vector<double>& v) {
        double q = 0; for (int i = 0; i < 3; ++i) { double Av = 0; for (int j = 0; j < 3; ++j) Av += A[i][j] * v[j]; q += 0.5 * v[i] * Av + b[i] * v[i]; }
        return q;
    };
    std::vector<double> grad(3); for (int i = 0; i < 3; ++i) { double Ax = 0; for (int j = 0; j < 3; ++j) Ax += A[i][j] * x[j]; grad[i] = Ax + b[i]; }

    auto err = [&](int N) { const auto g = zerothOrderGradient(f, x, 0.01, N, 7u);
        double e = 0; for (int i = 0; i < 3; ++i) e += (g[i] - grad[i]) * (g[i] - grad[i]); return std::sqrt(e); };
    const double eSmall = err(1000), eLarge = err(80000);
    const auto gL = zerothOrderGradient(f, x, 0.01, 80000, 7u);
    std::printf("zeroth_quadratic: grad=(%.3f,%.3f,%.3f) est=(%.3f,%.3f,%.3f) err(1k)=%.4f err(80k)=%.4f\n",
                grad[0], grad[1], grad[2], gL[0], gL[1], gL[2], eSmall, eLarge);
    TST_REQUIRE(eLarge < 0.02);            // large-N estimate ≈ analytic gradient
    TST_REQUIRE(eLarge < eSmall);          // variance falls with N
}

// --- (2) zeroth-order == analytic Dual gradient on the smooth pendulum objective ---------------
TST_CASE(physics, unit, diff_zeroth_order_matches_analytic) {
    const DiffModel md = pendModel();
    const V3<double> g{ 0, -9.81, 0 };
    const double h = 1.0 / 2000.0; const int steps = 150;
    const double q0 = 0.3, tau0 = 0.6;

    // Analytic gradient d(final angle)/d(tau) via forward-mode Dual.
    const Dual<1> outA = pendFinalAngle<Dual<1>>(md, q0, Dual<1>::seed(tau0, 0), g, h, steps);
    const double analytic = outA.d[0];

    // Zeroth-order estimate of the same derivative (1-D), forward evaluations only.
    auto f = [&](const std::vector<double>& v) { return pendFinalAngle<double>(md, q0, v[0], g, h, steps); };
    const auto g0 = zerothOrderGradient(f, std::vector<double>{ tau0 }, 1e-3, 40000, 11u);

    std::printf("zeroth_vs_analytic: analytic=%.8f zeroth=%.8f rel=%.4f%%\n",
                analytic, g0[0], 100.0 * std::fabs(g0[0] - analytic) / std::fabs(analytic));
    TST_REQUIRE(std::fabs(analytic) > 1e-4);
    TST_REQUIRE(std::fabs(g0[0] - analytic) / std::fabs(analytic) < 0.03);   // agree within Monte-Carlo error
}
