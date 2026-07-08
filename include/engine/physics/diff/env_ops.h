//
//  env_ops.h
//  engine::physics::diff
//
//  Shared RL-env operations for the diff ABA — the ACTUATION (action → joint torque) and the default
//  OBSERVATION packing — as `ENGINE_HD` free functions templated on the model type `M` and scalar `S`,
//  so the CPU `DiffVecEnv` and the CUDA `CudaVecEnv` call ONE implementation (no CPU/GPU divergence).
//  This is the wrapper-level counterpart of the model-generic ABA in articulated.h: previously the PD
//  servo and obs packing were hand-written a second time as device kernels in cuda_vec_env.cu, which
//  the code review flagged as a divergence risk (2026-07-08-cuda-port-code-review.md, MODERATE).
//
//  Observation layout (matches physics_env::Environment::packDefaultObs):
//    root pos(3) | root quat wxyz(4) | root linVel world(3) | root angVel world(3)
//    | joint q[ndof] | joint qd[ndof] | per-body contact flags[nBodies]
//  ⇒ obsDim = 13 + 2*ndof + nBodies.
//
//  Actuation (per joint, in joint coordinates): Torque = passthrough + per-DOF clamp; PDTarget =
//  kp·(target − q) − kd·q̇ clamped, with q the revolute angle (atan2 of the rest-relative rotation) or
//  the ball rotation-vector (SO(3) log). All math is scalar-generic (uses ADL std:: trig) so it runs
//  as double/float on host and float on device.
//

#pragma once

#include "engine/physics/diff/articulated.h"   // DiffState, HDLink, hd* accessors, LinkWorld, linkWorldInto

namespace engine::physics::diff {

// Action interpretation, as a plain int so device code needs no enum dependency:
//   0 = Torque (raw joint torques), 1 = PDTarget (desired joint pose tracked by a PD servo).
enum : int { kActionTorque = 0, kActionPDTarget = 1 };

template <class S> ENGINE_HD inline S clampAbs(S v, S m) { return v > m ? m : (v < -m ? -m : v); }

// Rotation matrix → quaternion (w,x,y,z), Shepperd's method (numerically stable branch on trace).
template <class S> ENGINE_HD inline void matToQuatWXYZ(const M3<S>& R, S& w, S& x, S& y, S& z) {
    using std::sqrt;
    const S tr = R.m[0][0] + R.m[1][1] + R.m[2][2];
    if (tr > S(0)) {
        S s = sqrt(tr + S(1)) * S(2);
        w = S(0.25) * s; x = (R.m[2][1] - R.m[1][2]) / s; y = (R.m[0][2] - R.m[2][0]) / s; z = (R.m[1][0] - R.m[0][1]) / s;
    } else if (R.m[0][0] > R.m[1][1] && R.m[0][0] > R.m[2][2]) {
        S s = sqrt(S(1) + R.m[0][0] - R.m[1][1] - R.m[2][2]) * S(2);
        w = (R.m[2][1] - R.m[1][2]) / s; x = S(0.25) * s; y = (R.m[0][1] + R.m[1][0]) / s; z = (R.m[0][2] + R.m[2][0]) / s;
    } else if (R.m[1][1] > R.m[2][2]) {
        S s = sqrt(S(1) + R.m[1][1] - R.m[0][0] - R.m[2][2]) * S(2);
        w = (R.m[0][2] - R.m[2][0]) / s; x = (R.m[0][1] + R.m[1][0]) / s; y = S(0.25) * s; z = (R.m[1][2] + R.m[2][1]) / s;
    } else {
        S s = sqrt(S(1) + R.m[2][2] - R.m[0][0] - R.m[1][1]) * S(2);
        w = (R.m[1][0] - R.m[0][1]) / s; x = (R.m[0][2] + R.m[2][0]) / s; y = (R.m[1][2] + R.m[2][1]) / s; z = S(0.25) * s;
    }
}

// Revolute joint angle from its rest-relative rotation about `axis`: θ = atan2(axis·vee(R), (trR−1)/2).
template <class S> ENGINE_HD inline S revoluteAngle(const M3<S>& R, const V3<S>& axis) {
    using std::atan2;
    const V3<S> v = vee(R);
    const S sn = v.x * axis.x + v.y * axis.y + v.z * axis.z;
    const S cs = S(0.5) * (R.m[0][0] + R.m[1][1] + R.m[2][2] - S(1));
    return atan2(sn, cs);
}

// Ball-joint rotation vector = SO(3) log of the rest-relative rotation (axis·angle).
template <class S> ENGINE_HD inline V3<S> ballRotVec(const M3<S>& R) {
    using std::acos; using std::sin;
    S c = S(0.5) * (R.m[0][0] + R.m[1][1] + R.m[2][2] - S(1));
    c = c > S(1) ? S(1) : (c < S(-1) ? S(-1) : c);
    const S th = acos(c);
    const V3<S> v = vee(R);                       // = sinθ · axis
    if (th < S(1e-6)) return v;                   // small-angle: log ≈ vee(R)
    const S k = th / sin(th);
    return { k * v.x, k * v.y, k * v.z };
}

// Map one env's flat action vector → per-DOF joint torque `tau` (both length numDof), for the given
// model + current state. `actionMode` is kActionTorque or kActionPDTarget. Shared by CPU + GPU.
template <class M, class S>
ENGINE_HD void actionToTau(const M& md, const DiffState<S>& st, const S* action, S* tau,
                           int actionMode, S kp, S kd, S maxTorque) {
    const int n = hdNumLinks(md);
    for (int i = 0; i < n; ++i) {
        const HDLink L = hdLink(md, i);
        const int dof = L.dof;
        if (dof <= 0) continue;
        const int qi = L.qIndex;
        if (actionMode == kActionTorque) {
            for (int d = 0; d < dof; ++d) tau[qi + d] = clampAbs(action[qi + d], maxTorque);
        } else if (dof == 1) {
            const V3<S> ax = lift<S>(L.axes[0]);
            const S q = revoluteAngle(st.linkRot[i], ax);
            tau[qi] = clampAbs(kp * (action[qi] - q) - kd * st.qd[qi], maxTorque);
        } else {                                  // ball (rotation-vector coordinates)
            const V3<S> rv = ballRotVec(st.linkRot[i]);
            const S rvv[3] = { rv.x, rv.y, rv.z };
            for (int d = 0; d < dof; ++d)
                tau[qi + d] = clampAbs(kp * (action[qi + d] - rvv[d]) - kd * st.qd[qi + d], maxTorque);
        }
    }
}

// Pack one env's default observation vector `o` (length obsDim, see header) from its state. Shared.
template <class M, class S>
ENGINE_HD void packDefaultObs(const M& md, const DiffState<S>& st, S* o) {
    int p = 0;
    o[p++] = st.basePos.x; o[p++] = st.basePos.y; o[p++] = st.basePos.z;
    S w, x, y, z; matToQuatWXYZ(st.baseRot, w, x, y, z);
    o[p++] = w; o[p++] = x; o[p++] = y; o[p++] = z;
    const V3<S> lv = st.baseRot * lin(st.baseTwist);   // world-frame root linear velocity
    const V3<S> av = st.baseRot * ang(st.baseTwist);   // world-frame root angular velocity
    o[p++] = lv.x; o[p++] = lv.y; o[p++] = lv.z;
    o[p++] = av.x; o[p++] = av.y; o[p++] = av.z;
    const int n = hdNumLinks(md);
    for (int i = 0; i < n; ++i) {                       // joint positions, link order
        const HDLink L = hdLink(md, i);
        if (L.dof == 1)      { o[p++] = revoluteAngle(st.linkRot[i], lift<S>(L.axes[0])); }
        else if (L.dof == 3) { const V3<S> rv = ballRotVec(st.linkRot[i]); o[p++] = rv.x; o[p++] = rv.y; o[p++] = rv.z; }
    }
    for (int i = 0; i < n; ++i) {                       // joint velocities, same order
        const HDLink L = hdLink(md, i);
        if (L.dof == 1)      { o[p++] = st.qd[L.qIndex]; }
        else if (L.dof == 3) { o[p++] = st.qd[L.qIndex]; o[p++] = st.qd[L.qIndex + 1]; o[p++] = st.qd[L.qIndex + 2]; }
    }
    // per-body contact flags: 1 if any of the body's ground spheres penetrates the plane (y ≤ 0).
    LinkWorld<S> lw[kMaxLinks];
    linkWorldInto(md, st, lw);
    for (int i = 0; i < n; ++i) {
        int flag = 0;
        const int nc = hdContactCount(md, i);
        for (int k = 0; k < nc; ++k) {
            const V3<double> od = hdContactOffset(md, i, k);
            const double r = hdContactRadius(md, i, k);
            const V3<S> offw = lw[i].rot * V3<S>{ S(od.x), S(od.y), S(od.z) };
            const S pen = S(r) - (lw[i].pos.y + offw.y);
            if (pen > S(0)) { flag = 1; break; }
        }
        o[p++] = S(flag);
    }
}

// Observation dimension for a model: 13 (root pos3+quat4+linvel3+angvel3) + 2*ndof + nBodies.
template <class M> ENGINE_HD inline int defaultObsDim(const M& md) {
    return 13 + 2 * hdNumDof(md) + hdNumLinks(md);
}

} // namespace engine::physics::diff
