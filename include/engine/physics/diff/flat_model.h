//
//  flat_model.h
//  engine::physics::diff
//
//  Device-friendly, fully flat, fixed-size POD encoding of a DiffModel's CONSTANTS (topology +
//  inertia + joint properties + ground-contact geometry). This resolves the "dynamic topology"
//  GPU-port blocker: the authoring `DiffModel` (std::vector-backed, grown at bake time) is baked
//  ONCE on the host into this flat form, which is then uploadable to the GPU as a single constant
//  blob — no pointers, no std::vector, `std::is_trivially_copyable`. The per-substep kernel then
//  reads only this shared, read-only model plus per-env mutable state (DiffState), which is exactly
//  the SIMT shape (one thread/warp per env, the model broadcast from constant/shared memory).
//
//  SoA layout (one array per field), so a field is a contiguous run addressable by link index
//  (0..numLinks), DOF index (0..numDof), or a flattened contact-sphere slot. Two DiffModel contact
//  representations — the multi-point `contactPoints` and the single-COM `contactRadius` shorthand —
//  are NORMALIZED here into one flat sphere list (matching linkGroundContactWorld's union rule), so
//  the device kernel needs no per-link branching on which representation is in use. Joint damping is
//  likewise PRE-RESOLVED (per-joint override or the inherited global) so the kernel reads it directly.
//
//  Constants stay `double` (as in DiffModel); the forward device kernel casts to float when reading
//  (per the CUDA-port precision policy — float forward, double for offline grad-checks). See
//  notes/investigations/physics/2026-07-06-cuda-port-review.md.
//

#pragma once

#include <cassert>
#include <type_traits>

#include "engine/physics/diff/articulated.h"

namespace engine::physics::diff {

// Max flattened contact spheres across all links: the worst case is every link a box (8 corners).
inline constexpr int kMaxContactSpheres = kMaxLinks * 8;

// Flat, POD, fixed-size, trivially-copyable model constants — the GPU upload blob.
struct FlatModel {
    // --- dimensions / global flags (int, not bool, for a clean POD on device) ---
    int numLinks = 0;
    int numDof   = 0;                 // == DiffModel::ndofJoints
    int floating = 0;

    // --- per-link topology + inertia (SoA, index 0..numLinks) ---
    int        parent[kMaxLinks]{};
    int        dof[kMaxLinks]{};
    int        qIndex[kMaxLinks]{};
    double     mass[kMaxLinks]{};
    M3<double> Ic[kMaxLinks]{};
    V3<double> axes[kMaxLinks][3]{};
    V3<double> anchorP[kMaxLinks]{};
    V3<double> anchorC[kMaxLinks]{};
    M3<double> restRel[kMaxLinks]{};

    // --- per-joint passive properties (pre-resolved: jointDamping is the effective value) ---
    double jointDamping[kMaxLinks]{};
    double jointStiffness[kMaxLinks]{};
    double armature[kMaxLinks]{};

    // --- ground-contact geometry: spheres flattened, per-link [offset, offset+count) ---
    int        contactGround = 0;
    int        numContactSpheres = 0;
    int        contactOffset[kMaxLinks]{};
    int        contactCount[kMaxLinks]{};
    V3<double> contactSphereOffset[kMaxContactSpheres]{};   // link-local, from the COM
    double     contactSphereRadius[kMaxContactSpheres]{};

    // --- smoothed ground-plane params (copied verbatim from DiffModel) ---
    double groundK = 0, groundC = 0, groundBeta = 0, groundDampBeta = 0, groundMu = 0, frictionVref = 0;
    int    contactIntegration = 0;   // 0 = Explicit, 1 = SemiImplicit
};

// Bake a host DiffModel into the flat device-upload form. Normalizes the contact representation and
// pre-resolves joint damping; a rig exceeding the fixed caps trips an assert (bump the caps).
inline FlatModel flatten(const DiffModel& md) {
    assert(md.links.size() <= static_cast<size_t>(kMaxLinks) && md.ndofJoints <= kMaxDof);
    FlatModel f{};
    f.numLinks = static_cast<int>(md.links.size());
    f.numDof   = md.ndofJoints;
    f.floating = md.floating ? 1 : 0;

    int c = 0;   // running flat contact-sphere cursor
    for (int i = 0; i < f.numLinks; ++i) {
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        f.parent[i] = L.parent; f.dof[i] = L.dof; f.qIndex[i] = L.qIndex;
        f.mass[i] = L.mass; f.Ic[i] = L.Ic;
        for (int d = 0; d < 3; ++d) f.axes[i][d] = L.axes[d];
        f.anchorP[i] = L.anchorP; f.anchorC[i] = L.anchorC; f.restRel[i] = L.restRel;
        f.jointDamping[i]   = (L.jointDamping >= 0.0) ? L.jointDamping : md.jointDamping;   // resolve inheritance
        f.jointStiffness[i] = L.jointStiffness;
        f.armature[i]       = L.armature;

        // Contact spheres: multi-point `contactPoints` if present, else the COM `contactRadius`
        // shorthand — the same union rule linkGroundContactWorld applies (never both).
        f.contactOffset[i] = c;
        if (!L.contactPoints.empty()) {
            for (const ContactSphere& cs : L.contactPoints) {
                assert(c < kMaxContactSpheres);
                f.contactSphereOffset[c] = cs.offset; f.contactSphereRadius[c] = cs.radius; ++c;
            }
        } else if (L.contactRadius > 0.0) {
            assert(c < kMaxContactSpheres);
            f.contactSphereOffset[c] = V3<double>{ 0, 0, 0 }; f.contactSphereRadius[c] = L.contactRadius; ++c;
        }
        f.contactCount[i] = c - f.contactOffset[i];
    }
    f.numContactSpheres = c;

    f.contactGround   = md.contactGround ? 1 : 0;
    f.groundK         = md.groundK;
    f.groundC         = md.groundC;
    f.groundBeta      = md.groundBeta;
    f.groundDampBeta  = md.groundDampBeta;
    f.groundMu        = md.groundMu;
    f.frictionVref    = md.frictionVref;
    f.contactIntegration = (md.contactIntegration == ContactIntegration::SemiImplicit) ? 1 : 0;
    return f;
}

// The whole point of the flat form: it must be a byte-blob we can memcpy to the GPU.
static_assert(std::is_trivially_copyable_v<FlatModel>, "FlatModel must be uploadable as a POD blob");

// ---- uniform model access (device side) --------------------------------------------------------
// ENGINE_HD counterparts of the host DiffModel accessors in articulated.h — same interface, so the
// model-generic ABA (diffSubstep<M,S>, diffForwardDynamics<M,S>, ...) runs on the GPU with FlatModel.
// FlatModel already pre-resolved jointDamping and normalized the contact spheres, so these are direct
// field reads (no inheritance/branching) — exactly the SIMT-friendly shape.
ENGINE_HD inline int  hdNumLinks(const FlatModel& m) { return m.numLinks; }
ENGINE_HD inline int  hdNumDof(const FlatModel& m)   { return m.numDof; }
ENGINE_HD inline bool hdFloating(const FlatModel& m) { return m.floating != 0; }
ENGINE_HD inline bool hdContactGround(const FlatModel& m) { return m.contactGround != 0; }
ENGINE_HD inline bool hdContactSemiImplicit(const FlatModel& m) { return m.contactIntegration != 0; }
ENGINE_HD inline double hdGroundK(const FlatModel& m)        { return m.groundK; }
ENGINE_HD inline double hdGroundC(const FlatModel& m)        { return m.groundC; }
ENGINE_HD inline double hdGroundBeta(const FlatModel& m)     { return m.groundBeta; }
ENGINE_HD inline double hdGroundDampBeta(const FlatModel& m) { return m.groundDampBeta; }
ENGINE_HD inline double hdGroundMu(const FlatModel& m)       { return m.groundMu; }
ENGINE_HD inline double hdFrictionVref(const FlatModel& m)   { return m.frictionVref; }
ENGINE_HD inline int hdContactCount(const FlatModel& m, int i) { return m.contactCount[i]; }
ENGINE_HD inline V3<double> hdContactOffset(const FlatModel& m, int i, int k) { return m.contactSphereOffset[m.contactOffset[i] + k]; }
ENGINE_HD inline double hdContactRadius(const FlatModel& m, int i, int k) { return m.contactSphereRadius[m.contactOffset[i] + k]; }
ENGINE_HD inline HDLink hdLink(const FlatModel& m, int i) {
    HDLink h; h.parent = m.parent[i]; h.dof = m.dof[i]; h.qIndex = m.qIndex[i]; h.mass = m.mass[i];
    h.jointDamping = m.jointDamping[i]; h.jointStiffness = m.jointStiffness[i]; h.armature = m.armature[i];
    h.Ic = &m.Ic[i]; h.restRel = &m.restRel[i]; h.anchorP = &m.anchorP[i]; h.anchorC = &m.anchorC[i]; h.axes = m.axes[i];
    return h;
}

} // namespace engine::physics::diff
