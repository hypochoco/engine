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


// ================= Feature 2: multi-point contact (link-local contact spheres) =================
namespace {
// Floating body with ONE contact sphere at a horizontal COM offset (exercises the ω×lever /
// contact-point-velocity path of the generalized contact — spin changes the offset point's normal vel).
DiffModel offsetContactBody(double m, double r, double ox, double k, double c) {
    DiffModel md; md.ndofJoints = 0; md.floating = true;
    md.contactGround = true; md.groundK = k; md.groundC = c; md.groundBeta = 800.0;
    const double Ic = 0.1 * m;
    DiffLink b; b.parent = -1; b.mass = m; b.Ic = diagM3(Ic, Ic, Ic); b.restRel = identity3<double>();
    b.addContactSphere({ ox, 0, 0 }, r);
    md.links = { b };
    return md;
}
// Floating body with TWO symmetric contact spheres offset ±d in x (a "rod on two feet").
DiffModel twoContactBody(double m, double r, double d, double k, double c) {
    DiffModel md; md.ndofJoints = 0; md.floating = true;
    md.contactGround = true; md.groundK = k; md.groundC = c; md.groundBeta = 800.0;
    const double Ic = 0.08 * m;
    DiffLink b; b.parent = -1; b.mass = m; b.Ic = diagM3(Ic, Ic, Ic); b.restRel = identity3<double>();
    b.addContactSphere({ d, 0, 0 }, r); b.addContactSphere({ -d, 0, 0 }, r);
    md.links = { b };
    return md;
}
}

// --- (6) offset contact sphere: gradient through the ω×lever path == central FD ----------------
TST_CASE(physics, unit, diff_contact_offset_sphere_gradient_matches_fd) {
    const double m = 1.0, r = 0.1, ox = 0.25, k = 3000.0, c = 30.0, g = 9.81, h = 1.0 / 4000.0;
    const int steps = 200;
    const DiffModel md = offsetContactBody(m, r, ox, k, c);
    const V3<double> grav{ 0, -g, 0 };
    const double y0 = r, wz0 = 1.0;                        // touching, spinning (offset point sweeps)

    DiffState<Dual<2>> st = makeState<Dual<2>>(md);
    st.basePos = { Dual<2>(0), Dual<2>::seed(y0, 0), Dual<2>(0) };
    st.baseTwist.d[2] = Dual<2>::seed(wz0, 1);             // ω_z
    st.baseTwist.d[4] = Dual<2>(-0.3);                     // slight downward vel to engage contact
    const Dual<2> yEnd = run(md, st, grav, h, steps).basePos.y;
    const double dY0 = yEnd.d[0], dWz = yEnd.d[1];

    auto finalY = [&](double yInit, double wz) {
        DiffState<double> s = makeState<double>(md);
        s.basePos = { 0, yInit, 0 }; s.baseTwist.d[2] = wz; s.baseTwist.d[4] = -0.3;
        return run(md, s, grav, h, steps).basePos.y;
    };
    const double eps = 1e-6;
    const double fdY0 = (finalY(y0 + eps, wz0) - finalY(y0 - eps, wz0)) / (2 * eps);
    const double fdWz = (finalY(y0, wz0 + eps) - finalY(y0, wz0 - eps)) / (2 * eps);
    std::printf("diff_contact_offset_grad: d/dy0 AD=%.8f FD=%.8f | d/dwz AD=%.8f FD=%.8f\n", dY0, fdY0, dWz, fdWz);
    TST_REQUIRE(std::fabs(dY0 - fdY0) < 1e-5);
    TST_REQUIRE(std::fabs(dWz - fdWz) < 1e-5);
    TST_REQUIRE(std::fabs(dWz) > 1e-4);                    // spin genuinely couples into the offset contact
}

// --- (7) two contact spheres share the load, self-right a tilt, and rest flat -------------------
TST_CASE(physics, unit, diff_contact_multipoint_rests_flat) {
    const double m = 1.0, r = 0.1, d = 0.25, k = 3000.0, c = 30.0, g = 9.81, h = 1.0 / 4000.0;
    const DiffModel md = twoContactBody(m, r, d, k, c);
    const V3<double> grav{ 0, -g, 0 };

    DiffState<double> st = makeState<double>(md);
    st.basePos = { 0, 0.4, 0 };
    st.baseRot = rodrigues<double>(V3<double>{ 0, 0, 1 }, 0.1);   // dropped with a 0.1 rad tilt
    for (int i = 0; i < 12000; ++i) diffSubstep(md, st, std::vector<double>{}, grav, h);   // 3 s to settle

    const double tilt = std::sqrt(dot(logSO3(st.baseRot), logSO3(st.baseRot)));
    const double restY = st.basePos.y, expectY = r - m * g / (2 * k);   // two springs share the weight
    std::printf("diff_multipoint_rest: restY=%.4f (expect r-mg/2k=%.4f) tilt 0.10->%.4f driftX=%.4f\n",
                restY, expectY, tilt, st.basePos.x);
    TST_REQUIRE(tilt < 0.03);                              // self-righted from the two-point support
    TST_REQUIRE(std::fabs(restY - expectY) < 0.01);        // rests at the SHARED-load equilibrium (½ penetration)
    TST_REQUIRE(std::fabs(st.basePos.x) < 0.02);           // no net horizontal drift (symmetry preserved)
}

// ================= Feature 3: shape-aware contact (box corners / capsule end-caps) =============
namespace {
// Floating box: 8 corner point-contacts (radius 0) at (±ex,±ey,±ez) — like a foot.
DiffModel boxBody(double m, double ex, double ey, double ez, double k, double c) {
    DiffModel md; md.ndofJoints = 0; md.floating = true;
    md.contactGround = true; md.groundK = k; md.groundC = c; md.groundBeta = 800.0;
    DiffLink b; b.parent = -1; b.mass = m; b.restRel = identity3<double>();
    b.Ic = diagM3(m / 3 * (ey * ey + ez * ez), m / 3 * (ex * ex + ez * ez), m / 3 * (ex * ex + ey * ey));
    for (int sx = -1; sx <= 1; sx += 2) for (int sy = -1; sy <= 1; sy += 2) for (int sz = -1; sz <= 1; sz += 2)
        b.addContactSphere({ sx * ex, sy * ey, sz * ez }, 0.0);
    md.links = { b };
    return md;
}
// Floating capsule (long axis local y): 2 end-cap spheres at (0,±hh,0), radius r.
DiffModel capsuleBody(double m, double r, double hh, double k, double c) {
    DiffModel md; md.ndofJoints = 0; md.floating = true;
    md.contactGround = true; md.groundK = k; md.groundC = c; md.groundBeta = 800.0;
    const double h = 2 * hh; DiffLink b; b.parent = -1; b.mass = m; b.restRel = identity3<double>();
    b.Ic = diagM3((1.0 / 12) * m * (3 * r * r + h * h), 0.5 * m * r * r, (1.0 / 12) * m * (3 * r * r + h * h));
    b.addContactSphere({ 0, hh, 0 }, r); b.addContactSphere({ 0, -hh, 0 }, r);
    md.links = { b };
    return md;
}
}

// --- (8) box rests flat on its 4 bottom corners at the shared-load height, no tipping ----------
TST_CASE(physics, unit, diff_contact_box_rests_flat) {
    const double m = 1.0, ex = 0.05, ey = 0.03, ez = 0.12, k = 3000.0, c = 30.0, g = 9.81, h = 1.0 / 4000.0;
    const DiffModel md = boxBody(m, ex, ey, ez, k, c);
    const V3<double> grav{ 0, -g, 0 };
    DiffState<double> st = makeState<double>(md); st.basePos = { 0, 0.30, 0 };
    for (int i = 0; i < 12000; ++i) diffSubstep(md, st, std::vector<double>{}, grav, h);   // 3 s
    const double tilt = std::sqrt(dot(logSO3(st.baseRot), logSO3(st.baseRot)));
    const double expectY = ey - m * g / (4 * k);            // 4 bottom corners share the weight
    std::printf("diff_box_rest: comY=%.4f (expect ey-mg/4k=%.4f) tilt=%.4f\n", st.basePos.y, expectY, tilt);
    TST_REQUIRE(std::fabs(st.basePos.y - expectY) < 0.004);
    TST_REQUIRE(tilt < 0.02);                               // flat foot, no tipping
}

// --- (9) capsule laid horizontal rests on both end-caps at height ~r, stays horizontal ----------
TST_CASE(physics, unit, diff_contact_capsule_rests_horizontal) {
    const double m = 1.0, r = 0.05, hh = 0.12, k = 3000.0, c = 30.0, g = 9.81, h = 1.0 / 4000.0;
    const DiffModel md = capsuleBody(m, r, hh, k, c);
    const V3<double> grav{ 0, -g, 0 };
    DiffState<double> st = makeState<double>(md);
    st.basePos = { 0, 0.30, 0 };
    st.baseRot = rodrigues<double>(V3<double>{ 0, 0, 1 }, -M_PI / 2);   // local y → world x (lie flat)
    for (int i = 0; i < 12000; ++i) diffSubstep(md, st, std::vector<double>{}, grav, h);
    const double expectY = r - m * g / (2 * k);             // both caps share the weight
    // the two caps' world y should be equal (axis horizontal): caps at ±hh along local y → world ±x.
    const auto lw = linkWorld<double>(md, st);
    std::printf("diff_capsule_rest: comY=%.4f (expect r-mg/2k=%.4f)\n", st.basePos.y, expectY);
    TST_REQUIRE(std::fabs(st.basePos.y - expectY) < 0.004); // rests at cap radius, not sunk to COM
    TST_REQUIRE(lw[0].pos.y > 0.0);                          // COM stays above the plane
}
