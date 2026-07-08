//
//  diff_semiimplicit.cpp
//  engine::tst / physics / unit
//
//  Deep-dive testing round (2026-07-04) on the semi-implicit contact integration (Feature 4) added to
//  stabilize the humanoid, plus the compliant ground-contact FORCE model it drives. These are
//  characterization + correctness probes; several DOCUMENT current behavior we suspect is buggy — those
//  assertions are labelled and are tripwires to flip once the behavior is fixed (fixes deferred per the
//  testing-round plan). Findings written up in notes/investigations/2026-07-04-diff-semiimplicit-testing.md.
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/articulated.h"
#include "harness/harness.h"

using namespace engine::physics::diff;

namespace {

// Total mechanical energy (KE + PE) of a model's moving links (skips a fixed base). PE is measured
// from `yDatum` (choose below the lowest COM so E stays positive ⇒ energy ratios are meaningful).
double energy(const DiffModel& md, const DiffState<double>& st, double g, double yDatum = 0.0) {
    const auto lw = linkWorld<double>(md, st);
    double E = 0.0;
    for (size_t i = 0; i < md.links.size(); ++i) {
        const DiffLink& L = md.links[i];
        if (L.parent < 0 && !md.floating) continue;             // fixed base: static, no energy
        const double m = L.mass;
        const V3<double> v = lw[i].linVel, w = lw[i].angVel;
        const M3<double> Iw = lw[i].rot * L.Ic * transpose(lw[i].rot);
        const double KE = 0.5 * m * dot(v, v) + 0.5 * dot(w, Iw * w);
        const double PE = m * g * (lw[i].pos.y - yDatum);
        E += KE + PE;
    }
    return E;
}

// A minimal model just to carry the ground constants for a direct force probe.
DiffModel groundOnly(double k, double c, double beta, double mu) {
    DiffModel md; md.contactGround = true;
    md.groundK = k; md.groundC = c; md.groundBeta = beta; md.groundMu = mu; md.frictionVref = 0.01;
    return md;
}

} // namespace

// --- (A) FIXED: the compliant normal force is non-adhesive. The damping term is gated off on
//         separation (vn > 0) by sigmoid(−groundDampBeta·vn), so a penetrating, separating contact
//         never PULLS down (Fn ≥ 0 — a unilateral contact can only push). Compression damping (approach,
//         vn < 0) is intact ⇒ low-bounce settling preserved. Regression guard for finding 1. ----------
TST_CASE(physics, unit, diff_ground_force_non_adhesive) {
    const DiffModel md = groundOnly(/*k*/1.0e4, /*c*/150.0, /*beta*/120.0, /*mu*/0.8);
    const double r = 0.1, pen = 0.002;                          // 2 mm penetration (typical at rest)
    const V3<double> com{ 0, r - pen, 0 };                      // COM contact sphere (offset 0)
    const M3<double> Rw = identity3<double>();
    auto Fn = [&](double vy) {
        return lin(groundContactSpatial(md, com, V3<double>{ 0, vy, 0 }, V3<double>{ 0, 0, 0 },
                                                Rw, V3<double>{ 0, 0, 0 }, r)).y;
    };
    const double Fn_rest = Fn(0.0);          // at rest: pure spring → repulsive
    const double Fn_up   = Fn(2.0);          // separating at 2 m/s → damping gated off
    const double Fn_down = Fn(-2.0);         // approaching at 2 m/s → damping adds repulsion
    std::printf("diff_non_adhesive: Fn(vn=-2)=%.2fN  Fn(vn=0)=%.2fN  Fn(vn=+2)=%.2fN\n", Fn_down, Fn_rest, Fn_up);
    TST_REQUIRE(Fn_rest > 0.0);                                 // resting contact pushes up
    TST_REQUIRE(Fn_up >= 0.0);                                  // FIXED: separating contact never pulls (non-adhesive)
    TST_REQUIRE(Fn_down > Fn_rest);                             // approach damping still adds repulsion (low bounce)
}

// --- (B) FIXED (IMEX): SemiImplicit now treats ONLY the stiff contact force implicitly and keeps the
//         smooth articulated dynamics explicit/symplectic, so it no longer numerically damps the whole
//         system. A frictionless pendulum is ~conserved under SemiImplicit, matching Explicit. Whole-
//         system damping, if wanted, is a separate PHYSICAL knob (jointDamping, test E). Regression
//         guard for finding 2. ---------------------------------------------------------------------
namespace {
DiffModel pendulum() {                                          // fixed base + 1 revolute link (about z)
    DiffModel md; md.ndofJoints = 1; md.floating = false; md.contactGround = false;
    DiffLink base; base.parent = -1; base.mass = 1.0; base.Ic = diagM3(0.01, 0.01, 0.01); base.restRel = identity3<double>();
    DiffLink link; link.parent = 0; link.dof = 1; link.qIndex = 0; link.mass = 1.0; link.Ic = diagM3(0.01, 0.01, 0.01);
    link.axes[0] = { 0, 0, 1 }; link.anchorP = { 0, 0, 0 }; link.anchorC = { 0, 0.5, 0 }; link.restRel = identity3<double>();
    md.links = { base, link };
    return md;
}
double energyRatioAfterSwing(ContactIntegration integ, double jointDamping, double h, int steps) {
    DiffModel md = pendulum(); md.contactIntegration = integ; md.jointDamping = jointDamping;
    DiffState<double> st = makeState<double>(md);
    st.linkRot[1] = rodrigues<double>(V3<double>{ 0, 0, 1 }, M_PI / 2);   // released horizontal, from rest
    const V3<double> grav{ 0, -9.81, 0 }; const double g = 9.81, datum = -0.6;
    const double E0 = energy(md, st, g, datum);
    for (int i = 0; i < steps; ++i) diffSubstep(md, st, std::vector<double>{ 0.0 }, grav, h);
    return energy(md, st, g, datum) / E0;
}
}
TST_CASE(physics, unit, diff_semiimplicit_imex_preserves_smooth_energy) {
    const double rExplicit = energyRatioAfterSwing(ContactIntegration::Explicit, 0.0, 1.0 / 240.0, 2400);
    const double rSemi     = energyRatioAfterSwing(ContactIntegration::SemiImplicit, 0.0, 1.0 / 240.0, 2400);
    std::printf("diff_imex_energy (contact-free pendulum, 10 s): E/E0 explicit=%.4f  semi-implicit(IMEX)=%.4f\n",
                rExplicit, rSemi);
    TST_REQUIRE(std::fabs(rSemi - rExplicit) < 0.02);   // IMEX: contact-only implicitness ⇒ smooth dynamics NOT damped
    TST_REQUIRE(rSemi > 0.9 && rExplicit > 0.9);        // both ~conserve (no artificial dissipation)
}

// --- (E) PHYSICAL joint damping: the optional `jointDamping` viscous term dissipates energy (the
//         intended way to damp the whole system), and being a real force it is ~timestep-independent
//         (halving h converges to the same trajectory) — unlike numerical/integrator damping. --------
TST_CASE(physics, unit, diff_joint_damping_physical) {
    const double rNone   = energyRatioAfterSwing(ContactIntegration::Explicit, 0.0, 1.0 / 240.0, 2400);
    const double rDamped = energyRatioAfterSwing(ContactIntegration::Explicit, 0.5, 1.0 / 240.0, 2400);
    const double rFineH  = energyRatioAfterSwing(ContactIntegration::Explicit, 0.5, 1.0 / 480.0, 4800);
    std::printf("diff_joint_damping: E/E0 b=0=%.4f  b=0.5=%.4f  b=0.5(h/2)=%.4f\n", rNone, rDamped, rFineH);
    TST_REQUIRE(rDamped < 0.5);                          // physical damping clearly dissipates
    TST_REQUIRE(rNone > 0.9);                            // no damping ⇒ ~conserved (isolates the effect)
    TST_REQUIRE(std::fabs(rDamped - rFineH) < 0.05);     // timestep-independent (physical, not an artifact)
}

// --- (C) CORRECTNESS: with a state-independent force (free fall, zero spin, no contact) the predictor
//         and corrector see the SAME acceleration, so SemiImplicit reduces EXACTLY to Explicit. -------
TST_CASE(physics, unit, diff_semiimplicit_equals_explicit_in_freefall) {
    auto drop = [&](ContactIntegration integ) {
        DiffModel md; md.ndofJoints = 0; md.floating = true; md.contactGround = false;
        md.contactIntegration = integ;
        DiffLink b; b.parent = -1; b.mass = 1.0; b.Ic = diagM3(0.01, 0.01, 0.01); b.restRel = identity3<double>();
        md.links = { b };
        DiffState<double> st = makeState<double>(md);
        st.basePos = { 0, 5, 0 }; st.baseTwist.d[4] = -0.3;      // pure vertical, zero angular velocity
        const V3<double> grav{ 0, -9.81, 0 };
        for (int i = 0; i < 500; ++i) diffSubstep(md, st, std::vector<double>{}, grav, 1.0 / 240.0);
        return st.basePos.y;
    };
    const double ye = drop(ContactIntegration::Explicit), ys = drop(ContactIntegration::SemiImplicit);
    std::printf("diff_semi_freefall: explicit y=%.10f semi y=%.10f  |diff|=%.2e\n", ye, ys, std::fabs(ye - ys));
    TST_REQUIRE(std::fabs(ye - ys) < 1e-12);     // identical when the force is state-independent
}

// --- (D) DETERMINISM: the semi-implicit contact path is bit-identical across runs. ------------------
TST_CASE(physics, unit, diff_semiimplicit_deterministic) {
    auto rollout = [&]() {
        DiffModel md; md.ndofJoints = 0; md.floating = true; md.contactGround = true;
        md.groundK = 2e4; md.groundC = 80; md.groundBeta = 400; md.contactIntegration = ContactIntegration::SemiImplicit;
        DiffLink b; b.parent = -1; b.mass = 1.0; b.Ic = diagM3(0.004, 0.004, 0.004); b.restRel = identity3<double>();
        b.contactRadius = 0.1; md.links = { b };
        DiffState<double> st = makeState<double>(md);
        st.basePos = { 0, 0.3, 0 }; st.baseTwist.d[3] = 0.5;    // drop + slide (engages friction too)
        const V3<double> grav{ 0, -9.81, 0 };
        for (int i = 0; i < 3000; ++i) diffSubstep(md, st, std::vector<double>{}, grav, 1.0 / 2000.0);
        return st;
    };
    const DiffState<double> a = rollout(), b = rollout();
    const double dy = std::fabs(a.basePos.y - b.basePos.y), dx = std::fabs(a.basePos.x - b.basePos.x);
    std::printf("diff_semi_determinism: |dx|=%.2e |dy|=%.2e\n", dx, dy);
    TST_REQUIRE(dx == 0.0 && dy == 0.0);
}

// ================= Per-joint properties (rig-agnostic; for authoring MJCF/URDF-style rigs) =========

// --- (F) Per-joint damping overrides the model-global jointDamping (<0 ⇒ inherit). ----------------
TST_CASE(physics, unit, diff_joint_damping_per_link) {
    auto ratio = [](double linkB, double modelB) {
        DiffModel md = pendulum(); md.jointDamping = modelB; md.links[1].jointDamping = linkB;
        DiffState<double> st = makeState<double>(md);
        st.linkRot[1] = rodrigues<double>(V3<double>{ 0, 0, 1 }, M_PI / 2);   // released horizontal
        const V3<double> grav{ 0, -9.81, 0 }; const double datum = -0.6;
        const double E0 = energy(md, st, 9.81, datum);
        for (int i = 0; i < 2400; ++i) diffSubstep(md, st, std::vector<double>{ 0.0 }, grav, 1.0 / 240.0);
        return energy(md, st, 9.81, datum) / E0;
    };
    const double perLink = ratio(0.5, 0.0);    // link damps, model-global off
    const double global  = ratio(-1.0, 0.5);   // link inherits (−1), model-global damps
    const double none    = ratio(-1.0, 0.0);   // link inherits, model-global off
    std::printf("diff_per_link_damping: perLink(b_link=0.5)=%.4f  global(inherit,b_model=0.5)=%.4f  none=%.4f\n",
                perLink, global, none);
    TST_REQUIRE(std::fabs(perLink - global) < 0.02);   // a per-link b matches the same b applied globally
    TST_REQUIRE(perLink < 0.5 && none > 0.9);          // damping dissipates; inherit-0 ~conserves
}

// --- (G) Passive joint stiffness pulls the joint back to its rest pose. ---------------------------
TST_CASE(physics, unit, diff_joint_stiffness_restores_to_rest) {
    auto finalSin = [](double k, double b) {
        DiffModel md = pendulum(); md.links[1].jointStiffness = k; md.links[1].jointDamping = b;
        DiffState<double> st = makeState<double>(md);
        st.linkRot[1] = rodrigues<double>(V3<double>{ 0, 0, 1 }, 0.6);   // displaced 0.6 rad from rest
        const V3<double> grav{ 0, 0, 0 };                                // NO gravity: isolate the spring
        for (int i = 0; i < 2400; ++i) diffSubstep(md, st, std::vector<double>{ 0.0 }, grav, 1.0 / 240.0);
        return vee(st.linkRot[1]).z;                                     // sinθ (≈ joint angle)
    };
    const double withSpring = finalSin(20.0, 1.0);   // stiff + damped → returns to rest
    const double noSpring   = finalSin(0.0, 1.0);    // no spring → stays displaced (damping can't restore)
    std::printf("diff_stiffness_restore: sin(theta) final  withSpring=%.4f  noSpring=%.4f (start=%.4f)\n",
                withSpring, noSpring, std::sin(0.6));
    TST_REQUIRE(std::fabs(withSpring) < 0.02);                          // spring pulled the joint back to rest
    TST_REQUIRE(std::fabs(noSpring - std::sin(0.6)) < 0.02);            // without a spring it stays put
}

// --- (H) Armature adds rotor/reflected inertia: tau/qddot = I_eff + armature (exact). --------------
TST_CASE(physics, unit, diff_armature_adds_rotor_inertia) {
    auto qddot = [](double armature) {
        DiffModel md = pendulum(); md.links[1].armature = armature;
        DiffState<double> st = makeState<double>(md);                   // at rest
        const Accel<double> a = diffForwardDynamics(md, st, std::vector<double>{ 3.0 }, V3<double>{ 0, 0, 0 });
        return a.qddot[0];
    };
    const double a0 = qddot(0.0), aA = qddot(0.05);
    const double Ieff0 = 3.0 / a0, IeffA = 3.0 / aA;
    std::printf("diff_armature: qddot 0=%.4f 0.05=%.4f  I_eff %.4f->%.4f (delta=%.6f, expect 0.05)\n",
                a0, aA, Ieff0, IeffA, IeffA - Ieff0);
    TST_REQUIRE(aA < a0);                                               // armature reduces acceleration
    TST_REQUIRE(std::fabs((IeffA - Ieff0) - 0.05) < 1e-9);              // exact rotor-inertia add to the joint-space inertia
}

// --- (I) Passive stiffness stays differentiable (uses vee ⇒ smooth): gradient == FD. --------------
TST_CASE(physics, unit, diff_joint_stiffness_differentiable) {
    DiffModel md = pendulum(); md.links[1].jointStiffness = 15.0; md.links[1].jointDamping = 0.2;
    const V3<double> grav{ 0, -9.81, 0 }; const double h = 1.0 / 1000.0; const int steps = 150;
    const double qd0 = 0.5;
    DiffState<Dual<1>> sd = makeState<Dual<1>>(md); sd.qd[0] = Dual<1>::seed(qd0, 0);
    for (int i = 0; i < steps; ++i) diffSubstep(md, sd, std::vector<Dual<1>>{ Dual<1>(0) }, grav, h);
    const double ad = vee(sd.linkRot[1]).z.d[0];
    auto finalSin = [&](double qd) {
        DiffState<double> st = makeState<double>(md); st.qd[0] = qd;
        for (int i = 0; i < steps; ++i) diffSubstep(md, st, std::vector<double>{ 0.0 }, grav, h);
        return vee(st.linkRot[1]).z;
    };
    const double eps = 1e-6, fd = (finalSin(qd0 + eps) - finalSin(qd0 - eps)) / (2 * eps);
    std::printf("diff_stiffness_grad: d(sinθ)/d(qd0) AD=%.8f FD=%.8f\n", ad, fd);
    TST_REQUIRE(std::fabs(ad - fd) < 1e-6);
}
