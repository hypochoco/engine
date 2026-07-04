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
std::pair<bool, double> dropHumanoid(double k, ContactIntegration integ, int substeps) {
    DiffModel md = articulationToDiffModel(physics::makeHumanoid(), DiffContact::All);
    md.groundK = k; md.contactIntegration = integ;
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

// (1) Stiffness sweep: SemiImplicit stays stable + low-penetration where Explicit fails. The payoff
// is at LOW substep counts (fewer substeps = cheaper) where the explicit stiff spring goes unstable.
TST_CASE(physics, integration, diff_contact_semiimplicit_stability) {
    for (int substeps : { 16, 48 }) {
        std::printf("humanoid drop, substeps=%d — Explicit vs SemiImplicit vs stiffness k:\n", substeps);
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
        TST_REQUIRE(lastSemi >= lastExplicit);          // semi-implicit is at least as stable everywhere
    }
    // The SHIPPED default (groundK=1e4, explicit) is stable + low-penetration even at the cheap
    // substeps=16 — Feature 3's multi-point contact conditioned the humanoid so explicit suffices.
    const auto [stable, pen] = dropHumanoid(1.0e4, ContactIntegration::Explicit, 16);
    std::printf("  shipped default (k=1e4, explicit, substeps=16): stable=%d pen=%.4f\n", stable, pen);
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
