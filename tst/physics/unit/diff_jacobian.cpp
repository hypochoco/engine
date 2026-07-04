//
//  diff_jacobian.cpp
//  engine::tst / physics / unit
//
//  Phase F3b: validate the per-step tangent-space state Jacobian ∂s_{t+1}/∂(s_t,a_t) (jacobian.h)
//  against tangent-space central finite differences, on a floating-base + revolute model that
//  exercises every block: base rotation, base translation, base twist, joint config, joint velocity,
//  and action. Perturbations/readouts of orientation use the SO(3) exp/log maps, so the comparison
//  is apples-to-apples in the Lie-algebra tangent.
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/jacobian.h"
#include "harness/harness.h"

using namespace engine::physics::diff;

namespace {

// floating sphere base + one revolute link.
DiffModel floatingRevolute() {
    DiffModel md; md.ndofJoints = 1; md.floating = true;
    DiffLink base; base.parent = -1; base.mass = 2; base.Ic = diagM3(0.05, 0.05, 0.05); base.restRel = identity3<double>();
    DiffLink link; link.parent = 0; link.dof = 1; link.qIndex = 0; link.mass = 1; link.Ic = diagM3(0.02, 0.02, 0.02);
    link.axes[0] = { 0, 0, 1 }; link.anchorP = { 0.3, 0, 0 }; link.anchorC = { -0.3, 0, 0 }; link.restRel = identity3<double>();
    md.links = { base, link };
    return md;
}

// Advance a copy of state0 by one substep with input direction k perturbed by `eps` (state dirs
// perturb the state on its manifold; the action dir perturbs the torque).
DiffState<double> perturbStep(const DiffModel& md, const TangentLayout& L, const DiffState<double>& state0,
                              const std::vector<double>& action0, int k, double eps, const V3<double>& g, double h) {
    DiffState<double> st = state0;
    std::vector<double> tau = action0;
    if (L.floating && k >= L.baseRotOff && k < L.baseRotOff + 3) {
        V3<double> e{ 0, 0, 0 }; (&e.x)[k - L.baseRotOff] = eps;
        st.baseRot = expSO3(e) * state0.baseRot;
    } else if (L.floating && k >= L.baseTransOff && k < L.baseTransOff + 3) {
        (&st.basePos.x)[k - L.baseTransOff] += eps;
    } else if (k >= L.cfgOff && k < L.cfgOff + L.ndof) {
        for (size_t i = 0; i < md.links.size(); ++i) { const DiffLink& Lk = md.links[i];
            for (int d = 0; d < Lk.dof; ++d) if (L.cfgOff + Lk.qIndex + d == k) {
                V3<double> ax = Lk.axes[d]; st.linkRot[i] = state0.linkRot[i] * expSO3(V3<double>{ eps * ax.x, eps * ax.y, eps * ax.z }); } }
    } else if (L.floating && k >= L.twistOff && k < L.twistOff + 6) {
        st.baseTwist.d[k - L.twistOff] += eps;
    } else if (k >= L.velOff && k < L.velOff + L.ndof) {
        st.qd[static_cast<size_t>(k - L.velOff)] += eps;
    } else {   // action
        tau[static_cast<size_t>(k - L.nState)] += eps;
    }
    diffSubstep(md, st, tau, g, h);
    return st;
}

// Output tangent of `out` relative to the nominal `ref` (orientation via log maps).
std::vector<double> outputTangent(const DiffModel& md, const TangentLayout& L,
                                  const DiffState<double>& out, const DiffState<double>& ref) {
    std::vector<double> t(static_cast<size_t>(L.nState), 0.0);
    if (L.floating) {
        const V3<double> w = logSO3(out.baseRot * transpose(ref.baseRot));   // left tangent
        t[static_cast<size_t>(L.baseRotOff + 0)] = w.x; t[static_cast<size_t>(L.baseRotOff + 1)] = w.y; t[static_cast<size_t>(L.baseRotOff + 2)] = w.z;
        t[static_cast<size_t>(L.baseTransOff + 0)] = out.basePos.x - ref.basePos.x;
        t[static_cast<size_t>(L.baseTransOff + 1)] = out.basePos.y - ref.basePos.y;
        t[static_cast<size_t>(L.baseTransOff + 2)] = out.basePos.z - ref.basePos.z;
        for (int j = 0; j < 6; ++j) t[static_cast<size_t>(L.twistOff + j)] = out.baseTwist.d[j] - ref.baseTwist.d[j];
    }
    for (size_t i = 0; i < md.links.size(); ++i) { const DiffLink& Lk = md.links[i];
        if (Lk.dof <= 0) continue;
        const V3<double> phi = logSO3(transpose(ref.linkRot[i]) * out.linkRot[i]);   // right tangent
        for (int d = 0; d < Lk.dof; ++d) t[static_cast<size_t>(L.cfgOff + Lk.qIndex + d)] = phi.x * Lk.axes[d].x + phi.y * Lk.axes[d].y + phi.z * Lk.axes[d].z; }
    for (int m = 0; m < L.ndof; ++m) t[static_cast<size_t>(L.velOff + m)] = out.qd[static_cast<size_t>(m)] - ref.qd[static_cast<size_t>(m)];
    return t;
}

} // namespace

TST_CASE(physics, unit, diff_jacobian_matches_fd) {
    const DiffModel md = floatingRevolute();
    const V3<double> g{ 0, -9.81, 0 };
    const double h = 1.0 / 2000.0;
    const TangentLayout L = tangentLayout(md);
    TST_REQUIRE(L.nState == 14 && L.nAction == 1 && L.nInput == 15);   // 12 base + 2 joint; 1 torque

    // A general (non-trivial) state so every block is exercised.
    DiffState<double> s0 = makeState<double>(md);
    s0.baseRot = expSO3(V3<double>{ 0.1, 0.2, -0.15 });
    s0.basePos = { 0.3, 0.5, -0.2 };
    s0.baseTwist = { { 0.5, -0.3, 0.4, 0.2, 0.1, -0.1 } };
    s0.linkRot[1] = rodrigues<double>(V3<double>{ 0, 0, 1 }, 0.4);
    s0.qd[0] = 0.6;
    const std::vector<double> a0{ 0.8 };

    const StepJacobian jac = stepJacobian(md, s0, a0, g, h);

    DiffState<double> ref = s0; diffSubstep(md, ref, std::vector<double>{ a0 }, g, h);   // nominal next state
    const double eps = 1e-6;
    double maxErr = 0;
    for (int k = 0; k < L.nInput; ++k) {
        const auto tp = outputTangent(md, L, perturbStep(md, L, s0, a0, k, +eps, g, h), ref);
        const auto tm = outputTangent(md, L, perturbStep(md, L, s0, a0, k, -eps, g, h), ref);
        for (int row = 0; row < L.nState; ++row) {
            const double fd = (tp[static_cast<size_t>(row)] - tm[static_cast<size_t>(row)]) / (2 * eps);
            const double ad = jac.J[static_cast<size_t>(row) * L.nInput + k];
            maxErr = std::max(maxErr, std::fabs(ad - fd));
        }
    }
    std::printf("diff_jacobian: 14x15 tangent Jacobian vs FD maxErr=%.3e\n", maxErr);
    TST_REQUIRE(maxErr < 1e-4);
}
