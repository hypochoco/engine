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
#include <cassert>
#include <vector>

#include "engine/physics/diff/linalg.h"

namespace engine::physics::diff {

// Compile-time capacities for the fixed-size (heap-free, device-portable) hot path. The batched
// GPU kernel needs the per-substep working set to live in stack/registers, not the heap, so
// diffForwardDynamics + Accel are sized by these caps rather than allocating per call. Sized to fit
// the built-in rigs with headroom: makeHumanoid (14 links / 21 DOF), makeAMPHumanoid (15 / 28).
// A model exceeding these trips an assert (see diffForwardDynamics) — bump the caps for bigger rigs.
// Kept snug (not generously over-sized): the per-substep working set is O(kMaxLinks) spatial
// matrices, so on the GPU these become per-thread local memory — tight caps lower register/local
// pressure, and on the CPU they avoid needlessly zero-initializing unused links each call.
inline constexpr int kMaxLinks = 16;   // makeHumanoid 14, makeAMPHumanoid 15
inline constexpr int kMaxDof   = 32;   // makeHumanoid 21, makeAMPHumanoid 28

// A ground-contact sphere fixed in a link's COM frame (Feature 2, multi-point contact): center at
// `offset` from the COM, radius `radius`. `radius==0` ⇒ a point contact (e.g. a box corner).
struct ContactSphere { V3<double> offset{ 0, 0, 0 }; double radius = 0.0; };

// Contact-force time integration (Feature 4). Explicit: forces at start-of-substep (fast, but stiff
// contact needs tiny h). SemiImplicit: IMEX — the stiff CONTACT force is re-evaluated at the PREDICTED
// end-of-substep state (an approximate backward-Euler step for contact only ⇒ larger k for the same h),
// while the smooth articulated dynamics stay explicit/symplectic, so it does NOT inject numerical
// damping into the whole system (only the contact term is treated implicitly).
enum class ContactIntegration { Explicit, SemiImplicit };

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
    // Ground contact geometry. `contactRadius>0` is a shorthand for one sphere at the COM (kept for
    // back-compat); `contactPoints` adds arbitrary link-local spheres. The effective set is the union.
    double                     contactRadius = 0.0;
    std::vector<ContactSphere> contactPoints;
    // Per-joint actuation-independent properties (for authoring MJCF/URDF-style rigs). All default to a
    // no-op so existing models are unchanged; applied to each of the joint's DOFs.
    double jointDamping   = -1.0;   // viscous τ = −b·q̇; <0 ⇒ inherit DiffModel::jointDamping
    double jointStiffness = 0.0;    // passive spring τ = −k·q pulling the joint toward its rest pose
    double armature       = 0.0;    // rotor/reflected inertia added to the joint-space inertia diagonal
    void addContactSphere(V3<double> offset, double radius) { contactPoints.push_back({ offset, radius }); }
};
struct DiffModel {
    std::vector<DiffLink> links;            // topological: parent index < child index; base parent −1
    int  ndofJoints = 0;                    // Σ joint dof (excludes the base)
    bool floating = false;                  // free 6-DOF root?
    // Smoothed compliant ground plane (y=0, normal +y). Normal force = k·smoothRelu(pen)
    // − c·vₙ·σ(β·pen)·σ(−βv·vₙ); β sets the smooth-relu / activation sharpness. Differentiable everywhere.
    // Stiffness/damping history: k was 2.5e3 (single COM sphere, ill-conditioned) → 1e4 once Feature 3's
    // shape-aware multi-point contact distributed the load. A soft k=1e4, though, lets a hard drop compress
    // the spring ~11 cm (limbs visibly clip the floor) and store enough energy to bounce ~28 cm once contact
    // is honestly non-adhesive. Stiffened to **4e4** (transient penetration ~3 cm) with **groundC=1000**
    // (over-damped ⇒ the drop barely bounces), validated stable (explicit) to k=8e4 at substeps 16–48 in
    // diff_contact_stability.cpp. SemiImplicit (IMEX) adds headroom for yet stiffer / larger-h regimes.
    bool   contactGround = false;
    double groundK = 4.0e4;
    double groundC = 1000.0;       // normal (compression) damping — over-damped for low bounce, no adhesion
    double groundBeta = 120.0;
    // Velocity gate for the normal DAMPING term (s/m). Damping is switched off on SEPARATION
    // (vn>0) via sigmoid(−groundDampBeta·vn), so the compliant contact is never ADHESIVE (the net
    // normal force stays ≥ 0 — a unilateral contact can only push). Compression damping (approach,
    // vn<0) is unaffected, so low-bounce settling is preserved. Smooth ⇒ differentiable.
    double groundDampBeta = 40.0;
    double groundMu = 0.8;         // Coulomb friction coefficient
    double frictionVref = 0.01;    // tangential-velocity regularization (m/s) — smooth Coulomb
    // Optional EXPLICIT PHYSICAL joint damping (viscous, τ = −jointDamping·q̇). Physical (not a
    // numerical/integrator artifact) ⇒ timestep-independent, differentiable, and can be matched to the
    // production backend so the diff sim and the RL sim agree. Default 0 (no damping). This — NOT the
    // integrator — is the intended way to add whole-system damping for realism / settling free DOFs.
    double jointDamping = 0.0;
    // Contact-force time integration (Feature 4, now IMEX). Explicit: forces at start-of-substep.
    // SemiImplicit: IMEX — the stiff CONTACT force is evaluated at the PREDICTED end-of-substep state
    // (implicit ⇒ stable at higher k), while the smooth articulated dynamics stay explicit/symplectic
    // (so it does NOT numerically damp the whole system — only the contact term is implicit).
    ContactIntegration contactIntegration = ContactIntegration::Explicit;
};

// Configuration + velocity. `linkRot[i]` is joint i's local rotation (rest-relative); identity for
// base + fixed joints. `qd` is the flat joint velocity vector. Fixed-size + POD (no heap): the whole
// per-env state is a trivially-copyable blob — on the GPU one such struct per thread (or a batched
// SoA), and the SemiImplicit trial-state copy becomes a plain memcpy. Only [0,numLinks)/[0,numDof)
// are meaningful; makeState/liftState set the dims and initialize linkRot to identity.
template <class S> struct DiffState {
    V3<S> basePos{};
    M3<S> baseRot = identity3<S>();
    V6<S> baseTwist{};                      // [ω; v] in the base/body frame (floating base)
    int   numLinks = 0;                     // == DiffModel::links.size()
    int   numDof   = 0;                     // == DiffModel::ndofJoints
    M3<S> linkRot[kMaxLinks]{};             // per link (set to identity by makeState/liftState)
    S     qd[kMaxDof]{};                    // per joint DOF
};

template <class S> DiffState<S> makeState(const DiffModel& md) {
    DiffState<S> s;
    s.numLinks = static_cast<int>(md.links.size());
    s.numDof   = md.ndofJoints;
    assert(s.numLinks <= kMaxLinks && s.numDof <= kMaxDof);
    for (int i = 0; i < s.numLinks; ++i) s.linkRot[i] = identity3<S>();   // qd[] already zero-initialized
    return s;
}

// Lift a double configuration into scalar type S (constants; e.g. to seed a differentiable rollout).
template <class S> DiffState<S> liftState(const DiffState<double>& s) {
    DiffState<S> r;
    r.numLinks = s.numLinks; r.numDof = s.numDof;
    r.basePos = lift<S>(s.basePos); r.baseRot = lift<S>(s.baseRot); r.baseTwist = liftV6<S>(s.baseTwist);
    for (int i = 0; i < s.numLinks; ++i) r.linkRot[i] = lift<S>(s.linkRot[i]);
    for (int i = 0; i < s.numDof; ++i) r.qd[i] = S(s.qd[i]);
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

template <class S> struct Accel { S qddot[kMaxDof]{}; V6<S> baseAccel{}; };

// Per-link world COM position + world orientation + world linear/angular velocity (rendering /
// energy / observation readout). Defined here (not merely forward-declared) so the fixed-size
// contact/kinematics helpers below can hold LinkWorld<S> arrays by value.
template <class S> struct LinkWorld { V3<S> pos, linVel, angVel; M3<S> rot; };
template <class S> void linkWorldInto(const DiffModel& md, const DiffState<S>& st, LinkWorld<S>* out);
template <class S> std::vector<LinkWorld<S>> linkWorld(const DiffModel& md, const DiffState<S>& st);

// Fixed-size per-link ground-contact spatial forces (WORLD frame) — the heap-free counterpart of a
// std::vector<V6<S>>, so the IMEX substep allocates nothing. Only [0,numLinks) is meaningful.
template <class S> struct ContactForces { V6<S> f[kMaxLinks]{}; };

// Smoothed compliant ground contact (plane y=0, normal +y) for a sphere of `radius` fixed at link-
// local `localOffset` from the COM (Feature 2). Sphere center = com + Rw·offset; contact point =
// center − r·n. Normal: k·softplus_β(pen)/β − c·vₙ·σ(β·pen)·σ(−βv·vₙ). The trailing σ(−βv·vₙ) is the
// approach gate: damping acts on approach (vₙ<0) but switches off on separation (vₙ>0), so the net
// normal force stays ≥ 0 (a unilateral contact can only push — never adhesive). Friction: regularized
// Coulomb Ft = −μ·Fspring·slip/√(|slip|²+ε²) where slip is the CONTACT-POINT tangential velocity
// v_com + ω × ℓ, ℓ = (center−com) − r·n. Force acts at the contact point ⇒ it torques the link.
// Returned as a spatial force [τ; f] in the WORLD frame; smooth everywhere ⇒ differentiable.
template <class S>
V6<S> groundContactWorld(const DiffModel& md, const V3<S>& com, const V3<S>& linVelWorld,
                         const V3<S>& angVelWorld, const M3<S>& Rw, const V3<double>& localOffset, double radius) {
    using std::sqrt;
    const V3<S> off = Rw * lift<S>(localOffset);           // COM → sphere center (world)
    const S pen = S(radius) - (com.y + off.y);             // penetration depth (>0 below the plane)
    const V3<S> lever = off + V3<S>{ S(0), S(-radius), S(0) };   // contact point − COM (world), n=+y
    const V3<S> cpVel = linVelWorld + cross(angVelWorld, lever);  // contact-point velocity
    const S vn = cpVel.y;                                  // normal velocity (n = +y)
    const S b = S(md.groundBeta);
    const S sr = softplus(b * pen) / b;                    // smooth relu(pen) → spring compression
    const S gate = sigmoid(b * pen);                       // in-contact gate (penetration)
    const S approach = sigmoid(S(-md.groundDampBeta) * vn); // ≈1 approaching (vn<0), ≈0 separating (vn>0)
    const S Fspring = S(md.groundK) * sr;                  // elastic normal load (≥ 0)
    const S Fn = Fspring - S(md.groundC) * vn * gate * approach;  // net normal force (≥ 0, non-adhesive)
    const S eps = S(md.frictionVref);
    const S vtMag = sqrt(cpVel.x * cpVel.x + cpVel.z * cpVel.z + eps * eps);
    const S fscale = S(-md.groundMu) * Fspring * gate / vtMag;
    const V3<S> Fworld{ fscale * cpVel.x, Fn, fscale * cpVel.z };
    const V3<S> tauWorld = cross(lever, Fworld);           // τ = lever × F
    return makeV6(tauWorld, Fworld);
}

// As above but returned as a spatial force [τ; f] in the LINK frame (back-compat convenience).
template <class S>
V6<S> groundContactSpatial(const DiffModel& md, const V3<S>& com, const V3<S>& linVelWorld,
                           const V3<S>& angVelWorld, const M3<S>& Rw, const V3<double>& localOffset, double radius) {
    const V6<S> w = groundContactWorld(md, com, linVelWorld, angVelWorld, Rw, localOffset, radius);
    return makeV6(transpose(Rw) * ang(w), transpose(Rw) * lin(w));
}

// Sum of a link's ground contact spatial forces (WORLD frame) given its world kinematics. Mirrors the
// contact block of diffForwardDynamics: prefer the multi-point `contactPoints`, else the `contactRadius`
// COM-sphere shorthand (never both — avoids double-counting the COM contact).
template <class S>
V6<S> linkGroundContactWorld(const DiffModel& md, const DiffLink& L, const V3<S>& com,
                             const V3<S>& linVelWorld, const V3<S>& angVelWorld, const M3<S>& Rw) {
    V6<S> f{};
    if (!L.contactPoints.empty())
        for (const ContactSphere& cs : L.contactPoints) f = f + groundContactWorld(md, com, linVelWorld, angVelWorld, Rw, cs.offset, cs.radius);
    else if (L.contactRadius > 0.0)
        f = f + groundContactWorld(md, com, linVelWorld, angVelWorld, Rw, V3<double>{ 0, 0, 0 }, L.contactRadius);
    return f;
}

// Per-link ground contact spatial forces (WORLD frame) for a whole state — used by the IMEX contact
// integrator to evaluate the (stiff) contact force at the PREDICTED state. Zero for non-contact links.
template <class S>
ContactForces<S> computeContactForcesWorld(const DiffModel& md, const DiffState<S>& st) {
    ContactForces<S> out;
    if (!md.contactGround) return out;
    LinkWorld<S> lw[kMaxLinks];
    linkWorldInto<S>(md, st, lw);
    for (size_t i = 0; i < md.links.size(); ++i)
        out.f[i] = linkGroundContactWorld(md, md.links[i], lw[i].pos, lw[i].linVel, lw[i].angVel, lw[i].rot);
    return out;
}

// Generalized accelerations via ABA. Mirrors featherstone_world.cpp::computeAccelerations. If
// `extContactWorld` is provided (IMEX contact integrator), the per-link ground contact forces are
// taken from it (evaluated at the predicted state, WORLD frame) instead of the current state — a
// plain pointer to a [numLinks) array (e.g. ContactForces::f), heap-free.
template <class S>
Accel<S> diffForwardDynamics(const DiffModel& md, const DiffState<S>& st,
                             const std::vector<S>& tau, const V3<double>& gravity,
                             const V6<S>* extContactWorld = nullptr) {
    const int n = static_cast<int>(md.links.size());
    assert(n <= kMaxLinks && md.ndofJoints <= kMaxDof);   // fixed-size working set (heap-free)
    M6<S> Xup[kMaxLinks]{}, IA[kMaxLinks]{};
    V6<S> v[kMaxLinks]{}, c[kMaxLinks]{}, pA[kMaxLinks]{}, a[kMaxLinks]{};
    M3<S> Rw[kMaxLinks]{};
    V3<S> pos[kMaxLinks]{};
    std::array<V6<S>, 3> Scol[kMaxLinks]{}, U[kMaxLinks]{};
    std::array<S, 9> Dinv[kMaxLinks]{};
    std::array<S, 3> uv[kMaxLinks]{};
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
        if (md.contactGround) {                                       // compliant ground contact
            V6<S> fWorld{};
            if (extContactWorld) {
                fWorld = extContactWorld[i];  // IMEX: contact force @ predicted state
            } else if (L.contactRadius > 0.0 || !L.contactPoints.empty()) {
                const V3<S> linVelWorld = Rw[i] * lin(v[i]);
                const V3<S> angVelWorld = Rw[i] * ang(v[i]);
                fWorld = linkGroundContactWorld(md, L, pos[i], linVelWorld, angVelWorld, Rw[i]);
            }
            const V6<S> fLink = makeV6(transpose(Rw[i]) * ang(fWorld), transpose(Rw[i]) * lin(fWorld));
            pA[i] = pA[i] + scaled(fLink, S(-1));
        }
    }

    for (int i = n - 1; i >= 0; --i) {                              // Pass 2: tips → base
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        if (L.parent < 0) continue;
        M6<S> Ia = IA[i]; V6<S> pa = pA[i];
        const int dof = L.dof;
        if (dof > 0) {
            S D[9];
            const double bEff = (L.jointDamping >= 0.0) ? L.jointDamping : md.jointDamping;   // per-joint damping or inherited global
            V3<S> rotvec = zeroV3<S>();
            if (L.jointStiffness != 0.0) rotvec = vee(st.linkRot[static_cast<size_t>(i)]);   // sinθ·axis: smooth, zero at rest (passive spring)
            for (int aa = 0; aa < dof; ++aa) {
                U[i][aa] = IA[i] * Scol[i][aa];
                S gen = tau[static_cast<size_t>(L.qIndex + aa)] - S(bEff) * st.qd[static_cast<size_t>(L.qIndex + aa)];
                if (L.jointStiffness != 0.0) gen = gen - S(L.jointStiffness) * dot(rotvec, lift<S>(L.axes[aa]));   // −k·q toward rest
                uv[i][aa] = gen - dot(Scol[i][aa], pA[i]);
            }
            for (int aa = 0; aa < dof; ++aa) for (int bb = 0; bb < dof; ++bb) D[aa * dof + bb] = dot(Scol[i][aa], U[i][bb]);
            if (L.armature != 0.0) for (int aa = 0; aa < dof; ++aa) D[aa * dof + aa] = D[aa * dof + aa] + S(L.armature);   // rotor/reflected inertia
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

    Accel<S> res{};   // qddot[] zero-initialized; DOFs not written below stay zero
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

// Apply an acceleration to the velocities (joint qd + floating base twist).
template <class S>
void diffApplyAccel(const DiffModel& md, DiffState<S>& st, const Accel<S>& acc, double h) {
    for (int k = 0; k < md.ndofJoints; ++k) st.qd[static_cast<size_t>(k)] = st.qd[static_cast<size_t>(k)] + acc.qddot[static_cast<size_t>(k)] * S(h);
    if (md.floating) for (int k = 0; k < 6; ++k) st.baseTwist.d[k] = st.baseTwist.d[k] + acc.baseAccel.d[k] * S(h);
}

// Advance the configuration (joint rotations + base pose) on the manifold using the CURRENT
// velocities (SO(3) exp map) — the position half of a semi-implicit Euler step.
template <class S>
void diffIntegrateConfig(const DiffModel& md, DiffState<S>& st, double h) {
    for (int i = 0; i < static_cast<int>(md.links.size()); ++i) {
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        if (L.dof <= 0) continue;
        V3<S> wc = zeroV3<S>();
        for (int d = 0; d < L.dof; ++d) wc = wc + st.qd[static_cast<size_t>(L.qIndex + d)] * lift<S>(L.axes[d]);
        st.linkRot[static_cast<size_t>(i)] = st.linkRot[static_cast<size_t>(i)] * expSO3(S(h) * wc);
    }
    if (md.floating) {
        const V3<S> wWorld = st.baseRot * ang(st.baseTwist);
        const V3<S> vWorld = st.baseRot * lin(st.baseTwist);
        st.basePos = st.basePos + S(h) * vWorld;
        st.baseRot = expSO3(S(h) * wWorld) * st.baseRot;
    }
}

// One substep. Explicit: semi-implicit Euler (velocity update from start-of-step forces, then config
// advance). SemiImplicit (Feature 4, IMEX): predictor→corrector — take an explicit trial step, then
// re-evaluate ONLY the stiff CONTACT force at that predicted state and take the real step with the
// smooth dynamics at the current state + contact at the predicted state. So the contact term is
// treated implicitly (stable at higher k) while the smooth articulated dynamics stay explicit/
// symplectic (no numerical damping of the whole system — that was the pre-IMEX behavior).
template <class S>
void diffSubstep(const DiffModel& md, DiffState<S>& st, const std::vector<S>& tau,
                 const V3<double>& gravity, double h) {
    if (md.contactIntegration == ContactIntegration::SemiImplicit && md.contactGround) {
        DiffState<S> trial = st;                                             // predictor: explicit trial step
        diffApplyAccel(md, trial, diffForwardDynamics(md, st, tau, gravity), h);
        diffIntegrateConfig(md, trial, h);
        const ContactForces<S> fc = computeContactForcesWorld(md, trial);    // stiff contact @ predicted state
        const Accel<S> acc = diffForwardDynamics(md, st, tau, gravity, fc.f); // smooth @ st, contact @ predicted (IMEX)
        diffApplyAccel(md, st, acc, h);
        diffIntegrateConfig(md, st, h);
    } else {
        const Accel<S> acc = diffForwardDynamics(md, st, tau, gravity);
        diffApplyAccel(md, st, acc, h);
        diffIntegrateConfig(md, st, h);
    }
}

// Per-link world kinematics into a caller-provided [numLinks) array — heap-free, so the IMEX
// substep (via computeContactForcesWorld) allocates nothing. See the LinkWorld def near the top.
template <class S>
void linkWorldInto(const DiffModel& md, const DiffState<S>& st, LinkWorld<S>* out) {
    const int n = static_cast<int>(md.links.size());
    assert(n <= kMaxLinks);
    M3<S> Rw[kMaxLinks]{}; V3<S> pos[kMaxLinks]{}; V6<S> v[kMaxLinks]{};
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
        out[i] = { pos[i], Rw[i] * lin(v[i]), Rw[i] * ang(v[i]), Rw[i] };
    }
}

// Convenience std::vector wrapper for host-side readout (rendering / energy / observation). The hot
// path uses linkWorldInto directly.
template <class S>
std::vector<LinkWorld<S>> linkWorld(const DiffModel& md, const DiffState<S>& st) {
    std::vector<LinkWorld<S>> out(md.links.size());
    linkWorldInto<S>(md, st, out.data());
    return out;
}

} // namespace engine::physics::diff
