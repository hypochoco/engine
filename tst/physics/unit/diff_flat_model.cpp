#include "harness/harness.h"
//
//  diff_flat_model.cpp
//  engine::tst / physics / unit
//
//  The device-upload form of the differentiable model. Bakes the two production rigs (humanoid, AMP)
//  through articulationToDiffModel → flatten() and verifies the flat SoA FlatModel reproduces every
//  DiffModel constant faithfully: topology (parent/dof/qIndex), inertia (mass/Ic), joint frames
//  (axes/anchors/restRel), pre-resolved joint damping, and the normalized ground-contact sphere list
//  (multi-point contactPoints ∪ the COM contactRadius shorthand, flattened with per-link [offset,count)).
//  Also asserts FlatModel is trivially copyable — the property that lets it be memcpy'd to the GPU as a
//  single constant blob. This is the "dynamic topology → baked flat model" CUDA-port blocker, resolved.
//

#include <cstdio>
#include <type_traits>

#include "engine/physics/diff/articulated.h"
#include "engine/physics/diff/flat_model.h"
#include "engine/physics/diff/from_articulation.h"
#include "engine/physics/dynamics/articulation.h"

using namespace engine;
using namespace engine::physics::diff;

namespace {

bool eqV3(const V3<double>& a, const V3<double>& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
bool eqM3(const M3<double>& a, const M3<double>& b) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) if (a.m[i][j] != b.m[i][j]) return false;
    return true;
}

// Verify a flattened model reproduces its source DiffModel exactly. Returns via TST_REQUIRE.
void checkFidelity(const DiffModel& md) {
    const FlatModel f = flatten(md);

    TST_REQUIRE(f.numLinks == static_cast<int>(md.links.size()));
    TST_REQUIRE(f.numDof == md.ndofJoints);
    TST_REQUIRE((f.floating != 0) == md.floating);
    TST_REQUIRE(f.contactGround == (md.contactGround ? 1 : 0));

    // Ground-plane params copied verbatim.
    TST_REQUIRE(f.groundK == md.groundK && f.groundC == md.groundC && f.groundBeta == md.groundBeta);
    TST_REQUIRE(f.groundDampBeta == md.groundDampBeta && f.groundMu == md.groundMu && f.frictionVref == md.frictionVref);

    int expectedSpheres = 0;
    for (int i = 0; i < f.numLinks; ++i) {
        const DiffLink& L = md.links[static_cast<size_t>(i)];
        TST_REQUIRE(f.parent[i] == L.parent);
        TST_REQUIRE(f.dof[i] == L.dof);
        TST_REQUIRE(f.qIndex[i] == L.qIndex);
        TST_REQUIRE(f.mass[i] == L.mass);
        TST_REQUIRE(eqM3(f.Ic[i], L.Ic));
        TST_REQUIRE(eqM3(f.restRel[i], L.restRel));
        TST_REQUIRE(eqV3(f.anchorP[i], L.anchorP));
        TST_REQUIRE(eqV3(f.anchorC[i], L.anchorC));
        for (int d = 0; d < 3; ++d) TST_REQUIRE(eqV3(f.axes[i][d], L.axes[d]));

        // Joint damping is pre-resolved (per-joint override, else the inherited global).
        const double bEff = (L.jointDamping >= 0.0) ? L.jointDamping : md.jointDamping;
        TST_REQUIRE(f.jointDamping[i] == bEff);
        TST_REQUIRE(f.jointStiffness[i] == L.jointStiffness);
        TST_REQUIRE(f.armature[i] == L.armature);

        // Contact spheres: the union rule (contactPoints if any, else the COM contactRadius sphere).
        const int expectCount = !L.contactPoints.empty()
            ? static_cast<int>(L.contactPoints.size())
            : (L.contactRadius > 0.0 ? 1 : 0);
        TST_REQUIRE(f.contactCount[i] == expectCount);
        TST_REQUIRE(f.contactOffset[i] == expectedSpheres);
        for (int s = 0; s < expectCount; ++s) {
            const int slot = f.contactOffset[i] + s;
            if (!L.contactPoints.empty()) {
                TST_REQUIRE(eqV3(f.contactSphereOffset[slot], L.contactPoints[static_cast<size_t>(s)].offset));
                TST_REQUIRE(f.contactSphereRadius[slot] == L.contactPoints[static_cast<size_t>(s)].radius);
            } else {
                TST_REQUIRE(eqV3(f.contactSphereOffset[slot], V3<double>{ 0, 0, 0 }));
                TST_REQUIRE(f.contactSphereRadius[slot] == L.contactRadius);
            }
        }
        expectedSpheres += expectCount;
    }
    TST_REQUIRE(f.numContactSpheres == expectedSpheres);
}

} // namespace

TST_CASE(physics, unit, diff_flat_model) {
    static_assert(std::is_trivially_copyable_v<FlatModel>, "FlatModel must be a POD upload blob");

    // Humanoid: 14 links / 21 DOF, shape-aware contact on every body.
    const DiffModel humanoid = articulationToDiffModel(physics::makeHumanoid(), DiffContact::All);
    TST_REQUIRE(humanoid.links.size() == 14 && humanoid.ndofJoints == 21 && humanoid.floating);
    checkFidelity(humanoid);

    // AMP rig: 15 links / 28 DOF.
    const DiffModel amp = articulationToDiffModel(physics::makeAMPHumanoid(), DiffContact::All);
    TST_REQUIRE(amp.links.size() == 15 && amp.ndofJoints == 28 && amp.floating);
    checkFidelity(amp);

    // Both rigs fit the fixed caps, and the blob is a modest constant upload.
    const FlatModel fh = flatten(humanoid);
    TST_REQUIRE(fh.numLinks <= kMaxLinks && fh.numDof <= kMaxDof && fh.numContactSpheres <= kMaxContactSpheres);
    std::printf("FlatModel: humanoid links=%d dof=%d contactSpheres=%d | sizeof(FlatModel)=%zu bytes\n",
                fh.numLinks, fh.numDof, fh.numContactSpheres, sizeof(FlatModel));
}
