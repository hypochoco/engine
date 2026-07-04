//
//  articulated.h
//  engine::physics::diff
//
//  Scalar-generic Articulated-Body Algorithm (Phase F1c/F1d) — the differentiable counterpart of
//  the reduced backend's ABA (src/physics/backends/reduced/featherstone_world.cpp), templated on
//  the scalar so the SAME code runs as `double` (value) or `Dual<N>` (exact gradients). Handles a
//  FIXED or FLOATING base and Fixed(0)/Revolute(1)/Ball(3)-DOF rotation joints. Orientation is
//  carried as rotation matrices and advanced with the SO(3) exp map (no quaternions ⇒ smooth,
//  normalization-free), so everything is cleanly differentiable.
//
//  Configuration lives in `DiffState<S>` (base pose + twist + a per-link rotation matrix + a flat
//  joint velocity vector); constants live in the plain-`double` `DiffModel`.
//  See notes/investigations/2026-07-04-differentiable-reduced.md.
//

#pragma once

#include <array>
#include <vector>

#include "engine/physics/diff/linalg.h"

namespace engine::physics::diff {

struct DiffLink {
    double     mass = 1.0;
    M3<double> Ic{};                        // inertia about the COM, link frame
    int        parent = -1;                 // −1 for the base
    int        dof = 0;                     // 0 fixed / 1 revolute / 3 ball
    int        qIndex = -1;                 // start index in the joint velocity vector (dof > 0)
    V3<double> axes[3]{ {0,0,1}, {1,0,0}, {0,1,0} };   // child-frame DOF axes (revolute uses [0])
    V3<double> anchorP{0, 0, 0};            // joint anchor in the parent frame
    V3<double> anchorC{0, 0, 0};            // joint anchor in the child frame
    M3<double> restRel{};                   // rest child-in-parent rotation
    double     contactRadius = 0.0;         // >0 ⇒ a contact sphere at the COM (F2 ground contact)
};
struct DiffModel {
    std::vector<DiffLink> links;            // topological: parent index < child index; base parent −1
    int  ndofJoints = 0;                    // Σ joint dof (excludes the base)
    bool floating = false;                  // free 6-DOF root?
    // Smoothed compliant ground plane (y=0, normal +y). Normal force = k·smoothRelu(pen)
    // − c·vₙ·σ(β·pen); β sets the smooth-relu / activation sharpness. Differentiable everywhere.
    // Defaults SOFTENED (2026-07-04) for explicit-integrator stability at the env timestep: a lower
    // activation sharpness β and higher normal damping c tame the contact-onset force spike (the
    // dominant blow-up source through the articulated legs) with little effect on steady penetration.
    // A stiffer, penetration-accurate contact needs the deferred semi-implicit/implicit solver.
    bool   contactGround = false;
    double groundK = 2.5e3;
    double groundC = 80.0;
    double groundBeta = 120.0;
    double groundMu = 0.8;         // Coulomb friction coefficient
    double frictionVref = 0.01;    // tangential-velocity regularization (m/s) — smooth Coulomb
};

// Configuration + velocity. `linkRot[i]` is joint i's local rotation (rest-relative); identity for
// base + fixed joints. `qd` is the flat joint velocity vector (size ndofJoints).
template <class S> struct DiffState {
    V3<S> basePos{};
    M3<S> baseRot = identity3<S>();
    V6<S> baseTwist{};                      // [ω; v] in the base/body frame (floating base)
    std::vector<M3<S>> linkRot;             // per link (size = #links)
    std::vector<S> qd;                      // per joint DOF
};

template <class S> DiffState<S> makeState(const DiffModel& md) {
    DiffState<S> s;
    s.linkRot.assign(md.links.size(), identity3<S>());
    s.qd.assign(static_cast<size_t>(md.ndofJoints), S(0));
    return s;
}

// Lift a double configuration into scalar type S (constants; e.g. to seed a differentiable rollout).
template <class S> DiffState<S> liftState(const DiffState<double>& s) {
    DiffState<S> r;
    r.basePos = lift<S>(s.basePos); r.baseRot = lift<S>(s.baseRot); r.baseTwist = liftV6<S>(s.baseTwist);
    r.linkRot.resize(s.linkRot.size());
    for (size_t i = 0; i < s.linkRot.size(); ++i) r.linkRot[i] = lift<S>(s.linkRot[i]);
    r.qd.resize(s.qd.size());
    for (size_t i = 0; i < s.qd.size(); ++i) r.qd[i] = S(s.qd[i]);
    return r;
}

inline M3<double> diagM3(double a, double b, double c) {
    M3<double> r; r.m[0][0] = a; r.m[1][1] = b; r.m[2][2] = c; return r;
}
template <class S> V3<S> zeroV3() { return { S(0), S(0), S(0) }; }

// Motion subspace column for DOF `a`: S = [axis; −axis × anchorC] (child frame).
template <class S> V6<S> jointScol(const V3<S>& axis, const V3<S>& anchorC) {
    return makeV6(axis, S(-1) * cross(axis, anchorC));
}

template <class S> struct Accel { std::vector<S> qddot; V6<S> baseAccel; };

// Smoothed compliant ground contact (plane y=0, normal +y) for a sphere of `radius` at world COM
// `com` moving at world velocity `linVelWorld` / angular velocity `angVelWorld`. Normal:
// k·softplus_β(pen) − c·vₙ·σ(β·pen). Friction: regularized Coulomb Ft = −μ·Fspring·slip/√(|slip|²+ε²)
// where slip is the CONTACT-POINT tangential velocity v_com + ω×(−r·n) (so friction correctly
// vanishes at rolling). The force acts at the contact point ⇒ friction torques the sphere (it
// rolls). Returned as a spatial force [τ; f] in the LINK frame; smooth everywhere ⇒ differentiable.
template <class S>
V6<S> groundContactSpatial(const DiffModel& md, const V3<S>& com, const V3<S>& linVelWorld,
                           const V3<S>& angVelWorld, const M3<S>& Rw, double radius) {
    using std::sqrt;
    const S pen = S(radius) - com.y;                       // penetration depth (>0 below the plane)
    const S vn = linVelWorld.y;                            // normal velocity (n = +y)
    const S b = S(md.groundBeta);
    const S sr = softplus(b * pen) / b;                    // smooth relu(pen) → spring compression
    const S gate = sigmoid(b * pen);                       // damping/friction active only in contact
    const S Fspring = S(md.groundK) * sr;                  // elastic normal load (≥ 0)
    const S Fn = Fspring - S(md.groundC) * vn * gate;      // net normal force
    // Contact-point (slip) tangential velocity: v_com + ω × (−r·n), n=+y ⇒ (+ω_z r, ·, −ω_x r).
    const S slipX = linVelWorld.x + angVelWorld.z * S(radius);
    const S slipZ = linVelWorld.z - angVelWorld.x * S(radius);
    const S eps = S(md.frictionVref);
    const S vtMag = sqrt(slipX * slipX + slipZ * slipZ + eps * eps);
    const S fscale = S(-md.groundMu) * Fspring * gate / vtMag;
    const V3<S> Fworld{ fscale * slipX, Fn, fscale * slipZ };
    // Torque about the COM: lever = contactPoint − com = −r·n = (0,−r,0);  τ = lever × Fworld.
    const V3<S> tauWorld{ S(-radius) * Fworld.z, S(0), S(radius) * Fworld.x };
    return makeV6(transpose(Rw) * tauWorld, transpose(Rw) * Fworld);
}

// Generalized accelerations via ABA. Mirrors featherstone_world.cpp::computeAccelerations.
template <class S>
Accel<S> diffForwardDynamics(const DiffModel& md, const DiffState<S>& st,
                             const std::vector<S>& tau, const V3<double>& gravity) {
    const int n = static_cast<int>(md.links.size());
    std::vector<M6<S>> Xup(n), IA(n);
    std::vector<V6<S>> v(n), c(n), pA(n), a(n);
    std::vector<M3<S>> Rw(n);
    std::vector<V3<S>> pos(n);
    std::vector<std::array<V6<S>, 3>> Scol(n), U(n);
    std::vector<std::array<S, 9>> Dinv(n);
    std::vector<std::array<S, 3>> uv(n);
    const V3<S> g = lift<S>(gravity);

    for (int i = 0; i < n; ++i) {                                   // Pass 1: base → tips
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        if (L.parent < 0) {
            Rw[i] = st.baseRot; pos[i] = st.basePos;
            v[i] = md.floating ? st.baseTwist : V6<S>{};
            c[i] = V6<S>{};
        } else {
            const M3<S> R_cp = lift<S>(L.restRel) * st.linkRot[static_cast<size_t>(i)];
            const V3<S> p_cp = lift<S>(L.anchorP) - R_cp * lift<S>(L.anchorC);
            Xup[i] = plux(transpose(R_cp), p_cp);
            Rw[i] = Rw[L.parent] * R_cp;
            pos[i] = pos[L.parent] + Rw[L.parent] * p_cp;
            V6<S> vJ{};
            for (int d = 0; d < L.dof; ++d) {
                Scol[i][d] = jointScol(lift<S>(L.axes[d]), lift<S>(L.anchorC));
                vJ = vJ + scaled(Scol[i][d], st.qd[static_cast<size_t>(L.qIndex + d)]);
            }
            v[i] = Xup[i] * v[L.parent] + vJ;
            c[i] = (L.dof > 0) ? crm(v[i]) * vJ : V6<S>{};
        }
        IA[i] = spatialInertia(S(L.mass), lift<S>(L.Ic));
        const V6<S> crfIv = transpose(crm(v[i])) * (IA[i] * v[i]);
        const V6<S> fgrav = makeV6(zeroV3<S>(), transpose(Rw[i]) * (S(L.mass) * g));
        pA[i] = scaled(crfIv, S(-1)) + scaled(fgrav, S(-1));
        if (md.contactGround && L.contactRadius > 0.0) {            // smooth compliant ground contact
            const V3<S> linVelWorld = Rw[i] * lin(v[i]);
            const V3<S> angVelWorld = Rw[i] * ang(v[i]);
            pA[i] = pA[i] + scaled(groundContactSpatial(md, pos[i], linVelWorld, angVelWorld, Rw[i], L.contactRadius), S(-1));
        }
    }

    for (int i = n - 1; i >= 0; --i) {                              // Pass 2: tips → base
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        if (L.parent < 0) continue;
        M6<S> Ia = IA[i]; V6<S> pa = pA[i];
        const int dof = L.dof;
        if (dof > 0) {
            S D[9];
            for (int aa = 0; aa < dof; ++aa) { U[i][aa] = IA[i] * Scol[i][aa]; uv[i][aa] = tau[static_cast<size_t>(L.qIndex + aa)] - dot(Scol[i][aa], pA[i]); }
            for (int aa = 0; aa < dof; ++aa) for (int bb = 0; bb < dof; ++bb) D[aa * dof + bb] = dot(Scol[i][aa], U[i][bb]);
            S Di[9]; invertSmall(D, dof, Di);
            for (int aa = 0; aa < dof; ++aa) for (int bb = 0; bb < dof; ++bb) Dinv[i][static_cast<size_t>(aa * 3 + bb)] = Di[aa * dof + bb];
            for (int aa = 0; aa < dof; ++aa) for (int bb = 0; bb < dof; ++bb) Ia = Ia - outerScaled(U[i][aa], U[i][bb], Di[aa * dof + bb]);
            pa = pA[i] + Ia * c[i];
            for (int aa = 0; aa < dof; ++aa) { S du = S(0); for (int bb = 0; bb < dof; ++bb) du = du + Di[aa * dof + bb] * uv[i][bb]; pa = pa + scaled(U[i][aa], du); }
        }
        const M6<S> XT = transpose(Xup[i]);
        IA[L.parent] = IA[L.parent] + XT * Ia * Xup[i];
        pA[L.parent] = pA[L.parent] + XT * pa;
    }

    Accel<S> res; res.qddot.assign(static_cast<size_t>(md.ndofJoints), S(0)); res.baseAccel = V6<S>{};
    for (int i = 0; i < n; ++i) {                                   // Pass 3: base → tips
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        if (L.parent < 0) {
            if (md.floating) { a[i] = solveM6(IA[i], scaled(pA[i], S(-1))); res.baseAccel = a[i]; }
            else a[i] = V6<S>{};
            continue;
        }
        const V6<S> ap = Xup[i] * a[L.parent] + c[i];
        a[i] = ap;
        for (int aa = 0; aa < L.dof; ++aa) {
            S qdd = S(0); for (int bb = 0; bb < L.dof; ++bb) qdd = qdd + Dinv[i][static_cast<size_t>(aa * 3 + bb)] * (uv[i][bb] - dot(U[i][bb], ap));
            res.qddot[static_cast<size_t>(L.qIndex + aa)] = qdd;
            a[i] = a[i] + scaled(Scol[i][aa], qdd);
        }
    }
    return res;
}

// One semi-implicit Euler substep: integrate velocities, then advance configuration on the
// manifold (base pose + joint rotations) via the SO(3) exp map — matches the reduced backend.
template <class S>
void diffSubstep(const DiffModel& md, DiffState<S>& st, const std::vector<S>& tau,
                 const V3<double>& gravity, double h) {
    const Accel<S> acc = diffForwardDynamics(md, st, tau, gravity);
    for (int k = 0; k < md.ndofJoints; ++k) st.qd[static_cast<size_t>(k)] = st.qd[static_cast<size_t>(k)] + acc.qddot[static_cast<size_t>(k)] * S(h);
    if (md.floating) for (int k = 0; k < 6; ++k) st.baseTwist.d[k] = st.baseTwist.d[k] + acc.baseAccel.d[k] * S(h);

    for (int i = 0; i < static_cast<int>(md.links.size()); ++i) {   // integrate joint rotations
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        if (L.dof <= 0) continue;
        V3<S> wc = zeroV3<S>();
        for (int d = 0; d < L.dof; ++d) wc = wc + st.qd[static_cast<size_t>(L.qIndex + d)] * lift<S>(L.axes[d]);
        st.linkRot[static_cast<size_t>(i)] = st.linkRot[static_cast<size_t>(i)] * expSO3(S(h) * wc);
    }
    if (md.floating) {                                              // integrate base pose (world frame)
        const V3<S> wWorld = st.baseRot * ang(st.baseTwist);
        const V3<S> vWorld = st.baseRot * lin(st.baseTwist);
        st.basePos = st.basePos + S(h) * vWorld;
        st.baseRot = expSO3(S(h) * wWorld) * st.baseRot;
    }
}

// Per-link world COM position + world orientation + world linear/angular velocity (rendering /
// energy / observation readout).
template <class S> struct LinkWorld { V3<S> pos, linVel, angVel; M3<S> rot; };

template <class S>
std::vector<LinkWorld<S>> linkWorld(const DiffModel& md, const DiffState<S>& st) {
    const int n = static_cast<int>(md.links.size());
    std::vector<M3<S>> Rw(n); std::vector<V3<S>> pos(n); std::vector<V6<S>> v(n);
    std::vector<LinkWorld<S>> out(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        if (L.parent < 0) { Rw[i] = st.baseRot; pos[i] = st.basePos; v[i] = md.floating ? st.baseTwist : V6<S>{}; }
        else {
            const M3<S> R_cp = lift<S>(L.restRel) * st.linkRot[static_cast<size_t>(i)];
            const V3<S> p_cp = lift<S>(L.anchorP) - R_cp * lift<S>(L.anchorC);
            Rw[i] = Rw[L.parent] * R_cp;
            pos[i] = pos[L.parent] + Rw[L.parent] * p_cp;
            V6<S> vJ{};
            for (int d = 0; d < L.dof; ++d) vJ = vJ + scaled(jointScol(lift<S>(L.axes[d]), lift<S>(L.anchorC)), st.qd[static_cast<size_t>(L.qIndex + d)]);
            v[i] = plux(transpose(R_cp), p_cp) * v[L.parent] + vJ;
        }
        out[static_cast<size_t>(i)] = { pos[i], Rw[i] * lin(v[i]), Rw[i] * ang(v[i]), Rw[i] };
    }
    return out;
}

} // namespace engine::physics::diff
