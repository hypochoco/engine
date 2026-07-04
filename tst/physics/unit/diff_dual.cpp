//
//  diff_dual.cpp
//  engine::tst / physics / unit
//
//  Phase F1 kickoff: validate the forward-mode `Dual` AD type (chain rule vs known analytic
//  derivatives) and a first Scalar-generic differentiable dynamics — a revolute pendulum stepped
//  with semi-implicit Euler on its generalized coordinate (the 1-DOF ABA). Its gradients
//  d(final angle)/d(theta0, tau) via Dual must match central finite differences, and its motion
//  must match the analytic small-angle period. This pins the AD substrate + the FD-validation
//  methodology the full generic ABA (F1c) and smoothed contact (F2) will build on.
//

#include <array>
#include <cmath>
#include <cstdio>

#include "engine/physics/diff/dual.h"
#include "harness/harness.h"

using namespace engine::physics::diff;

namespace {

// A revolute pendulum's generalized-coordinate step (theta from straight-down equilibrium):
// H θ̈ = τ − m g L sin θ, integrated semi-implicit Euler (velocity then position) — exactly the
// reduced backend's 1-DOF ABA + integrator. Templated on the scalar so it runs as double OR Dual.
template <class S>
S pendulumFinalAngle(S theta0, S thetadot0, S tau,
                     double m, double L, double Ic, double g, double h, int steps) {
    using std::sin;                                  // Dual overload found via ADL; double via std
    const double H = m * L * L + Ic;                 // generalized inertia about the hinge (const)
    S theta = theta0, thetadot = thetadot0;
    for (int i = 0; i < steps; ++i) {
        const S tauG = S(-(m * g * L)) * sin(theta);           // gravity generalized torque
        const S acc  = (tau + tauG) * S(1.0 / H);
        thetadot = thetadot + acc * S(h);                      // semi-implicit: v then x
        theta    = theta + thetadot * S(h);
    }
    return theta;
}

} // namespace

// --- Dual: arithmetic + chain rule vs known analytic derivatives -------------------------------
TST_CASE(physics, unit, diff_dual_chain_rule) {
    // f(x) = x³ + 2x  ⇒  f'(x) = 3x² + 2. At x=2: f=12, f'=14.
    {
        const Dual<1> x = Dual<1>::seed(2.0, 0);
        const Dual<1> f = x * x * x + Dual<1>(2.0) * x;
        std::printf("dual f=x^3+2x @2: v=%.6f d=%.6f (expect 12, 14)\n", f.v, f.d[0]);
        TST_APPROX(f.v, 12.0, 1e-9);
        TST_APPROX(f.d[0], 14.0, 1e-9);
    }
    // g(x) = sin(x)·cos(x)  ⇒  g'(x) = cos²x − sin²x = cos(2x). At x=0.7.
    {
        const double xv = 0.7;
        const Dual<1> x = Dual<1>::seed(xv, 0);
        const Dual<1> g = sin(x) * cos(x);
        TST_APPROX(g.v, std::sin(xv) * std::cos(xv), 1e-9);
        TST_APPROX(g.d[0], std::cos(2 * xv), 1e-9);
    }
    // Two independent seeds: h(a,b) = a²·sin(b). ∂h/∂a = 2a·sin(b); ∂h/∂b = a²·cos(b).
    {
        const double av = 1.3, bv = 0.4;
        const Dual<2> a = Dual<2>::seed(av, 0);
        const Dual<2> b = Dual<2>::seed(bv, 1);
        const Dual<2> hh = a * a * sin(b);
        TST_APPROX(hh.v, av * av * std::sin(bv), 1e-9);
        TST_APPROX(hh.d[0], 2 * av * std::sin(bv), 1e-9);
        TST_APPROX(hh.d[1], av * av * std::cos(bv), 1e-9);
    }
    // sqrt derivative: d/dx sqrt(x) = 1/(2 sqrt(x)). At x=4 ⇒ 0.25.
    {
        const Dual<1> x = Dual<1>::seed(4.0, 0);
        const Dual<1> r = sqrt(x);
        TST_APPROX(r.v, 2.0, 1e-9);
        TST_APPROX(r.d[0], 0.25, 1e-9);
    }
}

// --- Differentiable pendulum: gradient via Dual == central finite differences ------------------
TST_CASE(physics, unit, diff_pendulum_gradient_matches_fd) {
    const double m = 1.0, L = 1.0, Ic = 0.0, g = 9.81, h = 1.0 / 2000.0;
    const int steps = 400;                               // 0.2 s
    const double th0 = 0.4, thd0 = 0.0, tauv = 0.7;

    // Analytic gradient: seed theta0 (index 0) and tau (index 1).
    const Dual<2> out = pendulumFinalAngle<Dual<2>>(
        Dual<2>::seed(th0, 0), Dual<2>(thd0), Dual<2>::seed(tauv, 1), m, L, Ic, g, h, steps);
    const double dTheta0 = out.d[0], dTau = out.d[1];

    // Central finite differences on the same generic function evaluated as double.
    auto val = [&](double a, double t) { return pendulumFinalAngle<double>(a, thd0, t, m, L, Ic, g, h, steps); };
    const double eps = 1e-6;
    const double fdTheta0 = (val(th0 + eps, tauv) - val(th0 - eps, tauv)) / (2 * eps);
    const double fdTau    = (val(th0, tauv + eps) - val(th0, tauv - eps)) / (2 * eps);

    std::printf("diff_pendulum grad: dTheta0 AD=%.8f FD=%.8f | dTau AD=%.8f FD=%.8f\n",
                dTheta0, fdTheta0, dTau, fdTau);
    // Exact AD vs O(eps²) central FD (double) ⇒ agree to well under 1e-5.
    TST_REQUIRE(std::fabs(dTheta0 - fdTheta0) < 1e-5);
    TST_REQUIRE(std::fabs(dTau - fdTau) < 1e-5);
    // Sanity: the value path reproduces itself and the gradients are non-trivial.
    TST_APPROX(out.v, val(th0, tauv), 1e-12);
    TST_REQUIRE(std::fabs(dTheta0) > 1e-3 && std::fabs(dTau) > 1e-4);
}

// --- Physical sanity: small-angle period matches 2π√(H/(m g L)) --------------------------------
TST_CASE(physics, unit, diff_pendulum_small_angle_period) {
    const double m = 1.0, L = 1.0, Ic = 0.0, g = 9.81, h = 1.0 / 4000.0;
    const double th0 = 0.05;                              // small angle
    const double H = m * L * L + Ic;
    const double expected = 2.0 * M_PI * std::sqrt(H / (m * g * L));   // ~2.006 s

    double theta = th0, thetadot = 0.0, t = 0.0, prev = th0;
    double tDown = -1, tUp = -1, minTheta = th0;
    for (int i = 0; i < 20000; ++i) {                    // up to 5 s
        const double tauG = -(m * g * L) * std::sin(theta);
        thetadot += (tauG / H) * h;
        theta += thetadot * h;
        t += h;
        minTheta = std::min(minTheta, theta);
        if (tDown < 0 && prev > 0 && theta <= 0) tDown = t;                 // first downward zero-cross (~T/4)
        else if (tDown > 0 && tUp < 0 && prev < 0 && theta >= 0) tUp = t;   // next upward zero-cross (~3T/4)
        prev = theta;
    }
    const double measured = 2.0 * (tUp - tDown);         // (3T/4 − T/4) = T/2
    std::printf("diff_pendulum period: measured=%.4f expected=%.4f minTheta=%.4f\n",
                measured, expected, minTheta);
    TST_REQUIRE(tDown > 0 && tUp > 0);
    TST_REQUIRE(std::fabs(measured - expected) / expected < 0.02);   // within 2%
    TST_REQUIRE(minTheta < -0.9 * th0);                              // swings ~symmetrically
}
