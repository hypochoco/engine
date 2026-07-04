//
//  jacobian.h
//  engine::physics::diff
//
//  Per-step state-transition Jacobian ∂s_{t+1}/∂(s_t, a_t) in TANGENT coordinates (Phase F3b) — the
//  boundary a downstream autodiff framework chains for BPTT. Computed by exact forward-mode AD, one
//  input direction at a time (a `Dual<1>` directional derivative per column). Orientation states
//  (base rotation, ball/revolute joint rotations) are perturbed and read out in the Lie-algebra
//  tangent (exp map in, `vee`/log out), so the Jacobian is a plain dense matrix over ℝ^nState.
//
//  Tangent state layout (floating base): [baseRot 3][baseTrans 3][jointCfg ndof] ‖ [baseTwist 6]
//  [jointVel ndof]. Fixed base drops the base blocks. Action = per-DOF joint torques (ndof).
//  See notes/investigations/2026-07-04-differentiable-reduced.md.
//

#pragma once

#include <vector>

#include "engine/physics/diff/articulated.h"

namespace engine::physics::diff {

struct TangentLayout {
    bool floating; int ndof;
    int baseRotOff, baseTransOff, cfgOff, twistOff, velOff;   // (base blocks valid only if floating)
    int nState, nAction, nInput;
};

inline TangentLayout tangentLayout(const DiffModel& md) {
    TangentLayout L{}; L.floating = md.floating; L.ndof = md.ndofJoints;
    int o = 0;
    if (md.floating) { L.baseRotOff = o; o += 3; L.baseTransOff = o; o += 3; }
    L.cfgOff = o; o += md.ndofJoints;
    if (md.floating) { L.twistOff = o; o += 6; }
    L.velOff = o; o += md.ndofJoints;
    L.nState = o; L.nAction = md.ndofJoints; L.nInput = L.nState + L.nAction;
    return L;
}

struct StepJacobian { int nState, nInput; std::vector<double> J; };   // row-major nState × nInput

// One control step's tangent Jacobian at (state0, action0). J[row*nInput + k] = ∂(output tangent
// component row)/∂(input tangent component k), where inputs are [state | action]. `nSubsteps`
// substeps of size `h` are chained inside the seeded pass (a full control step's Jacobian).
inline StepJacobian stepJacobian(const DiffModel& md, const DiffState<double>& state0,
                                 const std::vector<double>& action0, const V3<double>& gravity,
                                 double h, int nSubsteps = 1) {
    using D = Dual<1>;
    const TangentLayout L = tangentLayout(md);
    StepJacobian out; out.nState = L.nState; out.nInput = L.nInput;
    out.J.assign(static_cast<size_t>(L.nState) * L.nInput, 0.0);
    const int nlinks = static_cast<int>(md.links.size());

    for (int k = 0; k < L.nInput; ++k) {
        // --- build the state/action with input direction k seeded (derivative 1) ---
        DiffState<D> st;
        st.linkRot.resize(nlinks); st.qd.resize(md.ndofJoints);
        const D one = D::seed(0.0, 0);                    // value 0, derivative 1 (perturbation gen.)

        // base rotation (left perturbation: R = expSO3(δ)·R0)
        V3<D> drot{ D(0), D(0), D(0) };
        if (L.floating) for (int j = 0; j < 3; ++j) if (L.baseRotOff + j == k) { drot.x = (j == 0) ? one : D(0); drot.y = (j == 1) ? one : D(0); drot.z = (j == 2) ? one : D(0); }
        st.baseRot = expSO3(drot) * lift<D>(state0.baseRot);
        // base translation
        st.basePos = lift<D>(state0.basePos);
        if (L.floating) { if (L.baseTransOff + 0 == k) st.basePos.x = D::seed(state0.basePos.x, 0);
                          if (L.baseTransOff + 1 == k) st.basePos.y = D::seed(state0.basePos.y, 0);
                          if (L.baseTransOff + 2 == k) st.basePos.z = D::seed(state0.basePos.z, 0); }
        // base twist
        st.baseTwist = liftV6<D>(state0.baseTwist);
        if (L.floating) for (int j = 0; j < 6; ++j) if (L.twistOff + j == k) st.baseTwist.d[j] = D::seed(state0.baseTwist.d[j], 0);
        // joint rotations (right perturbation along the seeded DOF's axis) + joint velocities
        for (int i = 0; i < nlinks; ++i) {
            st.linkRot[static_cast<size_t>(i)] = lift<D>(state0.linkRot[static_cast<size_t>(i)]);
            const DiffLink& Lk = md.links[static_cast<size_t>(i)];
            for (int d = 0; d < Lk.dof; ++d) if (L.cfgOff + Lk.qIndex + d == k) {
                const V3<double>& ax = Lk.axes[d];
                const V3<D> delta{ one * D(ax.x), one * D(ax.y), one * D(ax.z) };
                st.linkRot[static_cast<size_t>(i)] = lift<D>(state0.linkRot[static_cast<size_t>(i)]) * expSO3(delta);
            }
        }
        for (int m = 0; m < md.ndofJoints; ++m)
            st.qd[static_cast<size_t>(m)] = (L.velOff + m == k) ? D::seed(state0.qd[static_cast<size_t>(m)], 0) : D(state0.qd[static_cast<size_t>(m)]);
        // action (torques)
        std::vector<D> tau(static_cast<size_t>(md.ndofJoints));
        for (int m = 0; m < md.ndofJoints; ++m)
            tau[static_cast<size_t>(m)] = (L.nState + m == k) ? D::seed(action0[static_cast<size_t>(m)], 0) : D(action0[static_cast<size_t>(m)]);

        // --- one control step (nSubsteps substeps), then read the output tangent (column k) ---
        for (int ss = 0; ss < nSubsteps; ++ss) diffSubstep(md, st, tau, gravity, h);
        auto put = [&](int row, double val) { out.J[static_cast<size_t>(row) * L.nInput + k] = val; };
        if (L.floating) {
            M3<double> R0, dR;
            for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) { R0.m[a][b] = st.baseRot.m[a][b].v; dR.m[a][b] = st.baseRot.m[a][b].d[0]; }
            const V3<double> w = vee(dR * transpose(R0));      // left tangent: skew(ω)=dR·R0ᵀ
            put(L.baseRotOff + 0, w.x); put(L.baseRotOff + 1, w.y); put(L.baseRotOff + 2, w.z);
            put(L.baseTransOff + 0, st.basePos.x.d[0]); put(L.baseTransOff + 1, st.basePos.y.d[0]); put(L.baseTransOff + 2, st.basePos.z.d[0]);
            for (int j = 0; j < 6; ++j) put(L.twistOff + j, st.baseTwist.d[j].d[0]);
        }
        for (int i = 0; i < nlinks; ++i) {
            const DiffLink& Lk = md.links[static_cast<size_t>(i)];
            if (Lk.dof <= 0) continue;
            M3<double> R0, dR;
            for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) { R0.m[a][b] = st.linkRot[static_cast<size_t>(i)].m[a][b].v; dR.m[a][b] = st.linkRot[static_cast<size_t>(i)].m[a][b].d[0]; }
            const V3<double> phi = vee(transpose(R0) * dR);    // right tangent: skew(φ)=R0ᵀ·dR
            for (int d = 0; d < Lk.dof; ++d) put(L.cfgOff + Lk.qIndex + d, phi.x * Lk.axes[d].x + phi.y * Lk.axes[d].y + phi.z * Lk.axes[d].z);
        }
        for (int m = 0; m < md.ndofJoints; ++m) put(L.velOff + m, st.qd[static_cast<size_t>(m)].d[0]);
    }
    return out;
}

} // namespace engine::physics::diff
