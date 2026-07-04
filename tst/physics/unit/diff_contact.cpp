//
//  diff_contact.cpp
//  engine::tst / physics / unit
//
//  Phase F2 (first increment): validate the smoothed compliant ground contact in the Scalar-generic
//  ABA. A floating sphere must (1) drop and rest on the compliant plane without tunneling, and
//  (2) yield gradients of its final height w.r.t. initial height + velocity — taken THROUGH active
//  contact via forward-mode `Dual` — that match central finite differences. This is the crux of the
//  differentiable-sim work: differentiating through contact. Friction + the radius lever are F2b.
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/articulated.h"
#include "harness/harness.h"

using namespace engine::physics::diff;

namespace {

// A single floating sphere (base only, no joints) with a contact sphere + compliant ground.
DiffModel fallingSphere(double m, double r, double k, double c) {
    DiffModel md; md.ndofJoints = 0; md.floating = true;
    md.contactGround = true; md.groundK = k; md.groundC = c; md.groundBeta = 800.0;
    const double Ic = 0.4 * m * r * r;
    DiffLink s; s.parent = -1; s.mass = m; s.Ic = diagM3(Ic, Ic, Ic); s.restRel = identity3<double>(); s.contactRadius = r;
    md.links = { s };
    return md;
}

template <class S>
DiffState<S> run(const DiffModel& md, DiffState<S> st, const V3<double>& g, double h, int steps) {
    const std::vector<S> tau;   // no joints
    for (int i = 0; i < steps; ++i) diffSubstep(md, st, tau, g, h);
    return st;
}

} // namespace

// --- (1) sphere drops and rests on the compliant plane (no tunneling) --------------------------
TST_CASE(physics, unit, diff_contact_sphere_rests) {
    const double m = 1.0, r = 0.2, k = 3000.0, c = 30.0, g = 9.81, h = 1.0 / 4000.0;
    const DiffModel md = fallingSphere(m, r, k, c);
    const V3<double> grav{ 0, -g, 0 };

    DiffState<double> st = makeState<double>(md);
    st.basePos = { 0, 0.6, 0 };                           // drop from 0.6 m
    double minY = st.basePos.y;
    for (int i = 0; i < 8000; ++i) { st = run(md, st, grav, h, 1); minY = std::min(minY, st.basePos.y); }
    const double finalY = st.basePos.y, finalVy = st.baseTwist.d[4];
    const double restPen = m * g / k;                     // compliant equilibrium penetration
    std::printf("diff_contact_rest: finalY=%.4f (r=%.2f, restY=%.4f) minY=%.4f vy=%.4e\n",
                finalY, r, r - restPen, minY, finalVy);
    TST_REQUIRE(std::fabs(finalY - (r - restPen)) < 0.01);   // rests at the compliant equilibrium
    TST_REQUIRE(minY > 0.0);                                 // never tunneled through the floor
    TST_REQUIRE(std::fabs(finalVy) < 0.05);                  // settled
}

// --- (2) gradient through ACTIVE contact == central finite differences -------------------------
TST_CASE(physics, unit, diff_contact_gradient_matches_fd) {
    const double m = 1.0, r = 0.2, k = 3000.0, c = 30.0, g = 9.81, h = 1.0 / 4000.0;
    const int steps = 200;                                // 0.05 s of ACTIVE contact
    const DiffModel md = fallingSphere(m, r, k, c);
    const V3<double> grav{ 0, -g, 0 };
    const double y0 = r, vy0 = -0.5;                      // just touching, moving down ⇒ contact engaged

    // Analytic: seed initial height (0) and initial vertical velocity (1).
    DiffState<Dual<2>> st = makeState<Dual<2>>(md);
    st.basePos = { Dual<2>(0), Dual<2>::seed(y0, 0), Dual<2>(0) };
    st.baseTwist.d[4] = Dual<2>::seed(vy0, 1);            // body-frame lin-y == world (baseRot = I)
    const Dual<2> yEnd = run(md, st, grav, h, steps).basePos.y;
    const double dY0 = yEnd.d[0], dVy0 = yEnd.d[1];

    auto finalY = [&](double yInit, double vyInit) {
        DiffState<double> s = makeState<double>(md);
        s.basePos = { 0, yInit, 0 }; s.baseTwist.d[4] = vyInit;
        return run(md, s, grav, h, steps).basePos.y;
    };
    const double eps = 1e-6;
    const double fdY0  = (finalY(y0 + eps, vy0) - finalY(y0 - eps, vy0)) / (2 * eps);
    const double fdVy0 = (finalY(y0, vy0 + eps) - finalY(y0, vy0 - eps)) / (2 * eps);
    std::printf("diff_contact_grad: d/dy0 AD=%.8f FD=%.8f | d/dvy0 AD=%.8f FD=%.8f\n", dY0, fdY0, dVy0, fdVy0);
    TST_REQUIRE(std::fabs(dY0 - fdY0) < 1e-5);
    TST_REQUIRE(std::fabs(dVy0 - fdVy0) < 1e-5);
    TST_REQUIRE(std::fabs(dY0) > 1e-3 && std::fabs(dVy0) > 1e-3);   // contact actually shapes the gradient
}

// --- (3) friction: a sliding sphere decelerates and spins up (rolling) --------------------------
TST_CASE(physics, unit, diff_contact_friction_rolls) {
    const double m = 1.0, r = 0.2, k = 3000.0, c = 30.0, g = 9.81, h = 1.0 / 4000.0;
    const DiffModel md = fallingSphere(m, r, k, c);        // groundMu = 0.8 (default)
    const V3<double> grav{ 0, -g, 0 };

    DiffState<double> st = makeState<double>(md);
    st.basePos = { 0, r, 0 }; st.baseTwist.d[3] = 2.0;     // touching, sliding +x at 2 m/s
    st = run(md, st, grav, h, 4000);                       // 1 s
    const auto w = linkWorld<double>(md, st);
    const double vxf = w[0].linVel.x, wz = w[0].angVel.z;
    std::printf("diff_contact_friction: vx 2.0->%.4f  omega_z=%.4f  (rolling ≈ -vx/r=%.4f)\n", vxf, wz, -vxf / r);
    TST_REQUIRE(vxf < 1.8 && vxf > 0.5);                   // friction decelerated it (toward rolling ~1.43)
    TST_REQUIRE(wz < -3.0);                                // and spun it up (rolls forward ⇒ ω_z < 0)
}

// --- (4) friction gradient through contact torque == central FD --------------------------------
TST_CASE(physics, unit, diff_contact_friction_gradient_matches_fd) {
    const double m = 1.0, r = 0.2, k = 3000.0, c = 30.0, g = 9.81, h = 1.0 / 4000.0;
    const int steps = 300;                                 // 0.075 s of sliding contact
    const DiffModel md = fallingSphere(m, r, k, c);
    const V3<double> grav{ 0, -g, 0 };
    const double vx0 = 1.5, y0 = r;

    DiffState<Dual<2>> st = makeState<Dual<2>>(md);
    st.basePos = { Dual<2>(0), Dual<2>::seed(y0, 1), Dual<2>(0) };
    st.baseTwist.d[3] = Dual<2>::seed(vx0, 0);
    const auto w = linkWorld<Dual<2>>(md, run(md, st, grav, h, steps));
    const Dual<2> vxEnd = w[0].linVel.x;
    const double dVx0 = vxEnd.d[0], dY0 = vxEnd.d[1];

    auto finalVx = [&](double vx, double y) {
        DiffState<double> s = makeState<double>(md);
        s.basePos = { 0, y, 0 }; s.baseTwist.d[3] = vx;
        return linkWorld<double>(md, run(md, s, grav, h, steps))[0].linVel.x;
    };
    const double eps = 1e-6;
    const double fdVx0 = (finalVx(vx0 + eps, y0) - finalVx(vx0 - eps, y0)) / (2 * eps);
    const double fdY0  = (finalVx(vx0, y0 + eps) - finalVx(vx0, y0 - eps)) / (2 * eps);
    std::printf("diff_contact_fric_grad: d(vxf)/d(vx0) AD=%.8f FD=%.8f | d(vxf)/d(y0) AD=%.8f FD=%.8f\n", dVx0, fdVx0, dY0, fdY0);
    TST_REQUIRE(std::fabs(dVx0 - fdVx0) < 1e-5);
    TST_REQUIRE(std::fabs(dY0 - fdY0) < 1e-5);
    TST_REQUIRE(std::fabs(dY0) > 1e-4);                    // the normal load (via y0) shapes friction
}

// --- (5) coupled hopper: floating base + revolute leg + foot contact; gradient == FD -----------
namespace {
DiffModel hopper() {
    DiffModel md; md.ndofJoints = 1; md.floating = true;
    md.contactGround = true; md.groundK = 4000.0; md.groundC = 40.0; md.groundBeta = 800.0; md.groundMu = 0.9;
    DiffLink base; base.parent = -1; base.mass = 2.0; base.Ic = diagM3(0.03, 0.03, 0.03); base.restRel = identity3<double>();
    DiffLink leg; leg.parent = 0; leg.dof = 1; leg.qIndex = 0; leg.mass = 1.0; leg.Ic = diagM3(0.006, 0.006, 0.006);
    leg.axes[0] = { 0, 0, 1 }; leg.anchorP = { 0, -0.15, 0 }; leg.anchorC = { 0, 0.25, 0 }; leg.restRel = identity3<double>();
    leg.contactRadius = 0.08;
    md.links = { base, leg };
    return md;
}
template <class S>
DiffState<S> runTau(const DiffModel& md, DiffState<S> st, const std::vector<S>& tau, const V3<double>& g, double h, int steps) {
    for (int i = 0; i < steps; ++i) diffSubstep(md, st, tau, g, h);
    return st;
}
}

TST_CASE(physics, unit, diff_contact_hopper_gradient_matches_fd) {
    const DiffModel md = hopper();
    const V3<double> grav{ 0, -9.81, 0 };
    const int steps = 400; const double h = 1.0 / 4000.0;   // 0.1 s (foot engages the ground)
    const double tau0 = 2.0, y0 = 0.5;

    DiffState<Dual<2>> st = makeState<Dual<2>>(md);
    st.basePos = { Dual<2>(0), Dual<2>::seed(y0, 1), Dual<2>(0) };
    const std::vector<Dual<2>> tau{ Dual<2>::seed(tau0, 0) };
    const Dual<2> yEnd = runTau(md, st, tau, grav, h, steps).basePos.y;
    const double dTau = yEnd.d[0], dY0 = yEnd.d[1];

    auto finalBaseY = [&](double t, double y) {
        DiffState<double> s = makeState<double>(md); s.basePos = { 0, y, 0 };
        return runTau(md, s, std::vector<double>{ t }, grav, h, steps).basePos.y;
    };
    const double eps = 1e-6;
    const double fdTau = (finalBaseY(tau0 + eps, y0) - finalBaseY(tau0 - eps, y0)) / (2 * eps);
    const double fdY0  = (finalBaseY(tau0, y0 + eps) - finalBaseY(tau0, y0 - eps)) / (2 * eps);
    std::printf("diff_contact_hopper: d(baseY)/d(hipTau) AD=%.8f FD=%.8f | d(baseY)/d(y0) AD=%.8f FD=%.8f\n", dTau, fdTau, dY0, fdY0);
    TST_REQUIRE(std::fabs(dTau - fdTau) < 1e-4);            // gradient thru joint+contact+floating base
    TST_REQUIRE(std::fabs(dY0 - fdY0) < 1e-4);
    TST_REQUIRE(std::fabs(dY0) > 1e-3);                     // initial height clearly propagates through contact
}

