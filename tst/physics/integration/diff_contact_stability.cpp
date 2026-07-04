//
//  diff_contact_stability.cpp
//  engine::tst / physics / integration
//
//  Phase F / Feature 4: semi-implicit contact integration. Compares Explicit vs SemiImplicit contact
//  on the real humanoid across ground stiffness `k` — SemiImplicit should stay stable (and low
//  penetration) at stiffnesses where Explicit blows up — and checks that gradients THROUGH the
//  semi-implicit contact still match finite differences (the integrator must stay differentiable).
//  See notes/investigations/2026-07-04-differentiable-contact-geometry.md (M3).
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/from_articulation.h"
#include "engine/physics/dynamics/articulation.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics::diff;

namespace {
// Drop the all-body-contact humanoid; return {stable, settled max contact penetration (m)}.
std::pair<bool, double> dropHumanoid(double k, ContactIntegration integ, int substeps, double c = 150.0) {
    DiffModel md = articulationToDiffModel(physics::makeHumanoid(), DiffContact::All);
    md.groundK = k; md.groundC = c; md.contactIntegration = integ;
    DiffState<double> st = makeState<double>(md); st.basePos = { 0, 0.99, 0 };
    const V3<double> grav{ 0, -9.81, 0 }; const double h = (1.0 / 60.0) / substeps;
    const std::vector<double> tau(static_cast<size_t>(md.ndofJoints), 0.0);
    bool finite = true;
    for (int c = 0; c < 250 && finite; ++c) {
        for (int i = 0; i < substeps; ++i) diffSubstep(md, st, tau, grav, h);
        for (const auto& w : linkWorld<double>(md, st)) finite = finite && std::isfinite(w.pos.y) && std::fabs(w.pos.y) < 10.0;
    }
    if (!finite) return { false, 0.0 };
    // deepest contact-point penetration at rest
    const auto lw = linkWorld<double>(md, st);
    double maxPen = 0;
    for (size_t i = 0; i < md.links.size(); ++i)
        for (const ContactSphere& cs : md.links[i].contactPoints) {
            const V3<double> center = lw[i].pos + lw[i].rot * cs.offset;
            maxPen = std::max(maxPen, cs.radius - center.y);
        }
    return { true, maxPen };
}
}

// (1) Stiffness sweep (fixed moderate damping C=150 to isolate the spring k). Reports the max stable
// k for Explicit vs SemiImplicit. NOTE: since the IMEX rewrite, SemiImplicit treats ONLY the contact
// force implicitly (the smooth dynamics stay explicit/symplectic — it no longer numerically damps the
// whole system), so it is correctness-focused and NOT strictly more stable than explicit at very coarse
// substeps; both must comfortably cover the shipped stiffness (k=4e4) at training substep counts.
TST_CASE(physics, integration, diff_contact_semiimplicit_stability) {
    for (int substeps : { 32, 48 }) {
        std::printf("humanoid drop, substeps=%d (C=150) — Explicit vs SemiImplicit vs stiffness k:\n", substeps);
        int lastExplicit = 0, lastSemi = 0;
        for (double k : { 2.5e3, 5e3, 1e4, 2e4, 4e4, 8e4 }) {
            const auto [se, pe] = dropHumanoid(k, ContactIntegration::Explicit, substeps);
            const auto [ss, ps] = dropHumanoid(k, ContactIntegration::SemiImplicit, substeps);
            std::printf("  k=%-7.0f explicit: stable=%d pen=%.4f | semi-implicit: stable=%d pen=%.4f\n",
                        k, se, se ? pe : 0.0, ss, ss ? ps : 0.0);
            if (se) lastExplicit = static_cast<int>(k);
            if (ss) lastSemi = static_cast<int>(k);
        }
        std::printf("  substeps=%d max stable k: explicit=%d semi-implicit=%d\n", substeps, lastExplicit, lastSemi);
        TST_REQUIRE(lastExplicit >= 40000 && lastSemi >= 40000);   // both cover the shipped k=4e4
    }
    // The SHIPPED default (k=4e4, C=1000, explicit) is stable + low settled penetration at the training
    // substep count (DiffEnvironment uses 48 for contact).
    const auto [stable, pen] = dropHumanoid(4.0e4, ContactIntegration::Explicit, 48, 1000.0);
    std::printf("  shipped default (k=4e4, C=1000, explicit, substeps=48): stable=%d pen=%.4f\n", stable, pen);
    TST_REQUIRE(stable && pen < 0.01);
}

// (2) Gradient THROUGH semi-implicit contact == central finite differences (must stay differentiable).
TST_CASE(physics, integration, diff_contact_semiimplicit_gradient_matches_fd) {
    // single floating sphere on stiff semi-implicit contact
    DiffModel md; md.ndofJoints = 0; md.floating = true;
    md.contactGround = true; md.groundK = 2e4; md.groundC = 60; md.groundBeta = 400;
    md.contactIntegration = ContactIntegration::SemiImplicit;
    DiffLink s; s.parent = -1; s.mass = 1; s.Ic = diagM3(0.004, 0.004, 0.004); s.restRel = identity3<double>(); s.contactRadius = 0.1;
    md.links = { s };
    const V3<double> grav{ 0, -9.81, 0 }; const double h = 1.0 / 4000.0; const int steps = 200;
    const double y0 = 0.1, vy0 = -0.4;

    auto finalY = [&](DiffState<double> st) { for (int i = 0; i < steps; ++i) diffSubstep(md, st, std::vector<double>{}, grav, h); return st.basePos.y; };
    DiffState<Dual<2>> sd = makeState<Dual<2>>(md);
    sd.basePos = { Dual<2>(0), Dual<2>::seed(y0, 0), Dual<2>(0) }; sd.baseTwist.d[4] = Dual<2>::seed(vy0, 1);
    for (int i = 0; i < steps; ++i) diffSubstep(md, sd, std::vector<Dual<2>>{}, grav, h);
    const double dY0 = sd.basePos.y.d[0], dVy0 = sd.basePos.y.d[1];

    auto mk = [&](double y, double vy) { DiffState<double> s0 = makeState<double>(md); s0.basePos = { 0, y, 0 }; s0.baseTwist.d[4] = vy; return finalY(s0); };
    const double eps = 1e-6;
    const double fdY0 = (mk(y0 + eps, vy0) - mk(y0 - eps, vy0)) / (2 * eps);
    const double fdVy0 = (mk(y0, vy0 + eps) - mk(y0, vy0 - eps)) / (2 * eps);
    std::printf("diff_semiimplicit_grad: d/dy0 AD=%.8f FD=%.8f | d/dvy0 AD=%.8f FD=%.8f\n", dY0, fdY0, dVy0, fdVy0);
    TST_REQUIRE(std::fabs(dY0 - fdY0) < 1e-5);
    TST_REQUIRE(std::fabs(dVy0 - fdVy0) < 1e-5);
}

// (3) Humanoid rest PARITY: at the shipped stiffness the semi-implicit and explicit integrators must
// settle the passive ragdoll to the SAME static equilibrium (contact spring balance is velocity-free at
// rest, so the integrator choice must not shift the resting pose / penetration). Also records the
// residual kinetic energy — semi-implicit's numerical damping should leave the ragdoll at least as
// settled as explicit (a system-level view of the contact-free-pendulum damping finding).
namespace {
struct Settle { double baseY, maxPen, residualKE; };
Settle settleHumanoid(ContactIntegration integ, int substeps) {
    DiffModel md = articulationToDiffModel(physics::makeHumanoid(), DiffContact::All);
    md.contactIntegration = integ;                             // shipped default contact (k=4e4, C=1000)
    DiffState<double> st = makeState<double>(md); st.basePos = { 0, 0.99, 0 };
    const V3<double> grav{ 0, -9.81, 0 }; const double h = (1.0 / 60.0) / substeps;
    const std::vector<double> tau(static_cast<size_t>(md.ndofJoints), 0.0);
    for (int c = 0; c < 300; ++c) for (int i = 0; i < substeps; ++i) diffSubstep(md, st, tau, grav, h);
    const auto lw = linkWorld<double>(md, st);
    double maxPen = 0.0, ke = 0.0;
    for (size_t i = 0; i < md.links.size(); ++i) {
        const DiffLink& L = md.links[i];
        const M3<double> Iw = lw[i].rot * L.Ic * transpose(lw[i].rot);
        ke += 0.5 * L.mass * dot(lw[i].linVel, lw[i].linVel) + 0.5 * dot(lw[i].angVel, Iw * lw[i].angVel);
        for (const ContactSphere& cs : L.contactPoints)
            maxPen = std::max(maxPen, cs.radius - (lw[i].pos + lw[i].rot * cs.offset).y);
    }
    return { st.basePos.y, maxPen, ke };
}
}
TST_CASE(physics, integration, diff_contact_semiimplicit_humanoid_parity) {
    const Settle e = settleHumanoid(ContactIntegration::Explicit, 48);
    const Settle s = settleHumanoid(ContactIntegration::SemiImplicit, 48);
    std::printf("humanoid rest parity (shipped k=4e4 C=1000, substeps=48): explicit baseY=%.4f pen=%.4f KE=%.3e | "
                "semi baseY=%.4f pen=%.4f KE=%.3e\n", e.baseY, e.maxPen, e.residualKE, s.baseY, s.maxPen, s.residualKE);
    TST_REQUIRE(std::isfinite(e.baseY) && std::isfinite(s.baseY));
    TST_REQUIRE(std::fabs(e.baseY - s.baseY) < 0.01);        // same static equilibrium (≤1 cm)
    TST_REQUIRE(std::fabs(e.maxPen - s.maxPen) < 0.005);     // same resting penetration
    TST_REQUIRE(s.maxPen < 0.01 && e.maxPen < 0.01);         // both rest cleanly (~mm)
    TST_REQUIRE(s.residualKE < 1e-2 && e.residualKE < 1e-2); // both effectively at rest (explicit jitters a touch more)
}
