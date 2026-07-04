//
//  amp_humanoid.cpp
//  engine::tst / physics / integration
//
//  Validate the AMP/DeepMimic humanoid preset (makeAMPHumanoid): correct topology (15 bodies /
//  28 DOF), converts cleanly to a differentiable DiffModel, and a passive drop stays finite and
//  rests ON the plane (no phase-through) under the shipped contact defaults. Windowed eyeball:
//  tst/physics/visual/amp_humanoid.cpp.
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/from_articulation.h"
#include "engine/physics/dynamics/articulation.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics::diff;

// (1) Topology: 15 bodies, 28 actuated DOF, floating pelvis root; converter accepts it.
TST_CASE(physics, integration, amp_humanoid_topology) {
    const physics::ArticulationDef def = physics::makeAMPHumanoid();
    TST_REQUIRE(def.bodies.size() == 15);
    TST_REQUIRE(def.joints.size() == 14);
    int dof = 0;
    for (const auto& j : def.joints)
        dof += (j.type == physics::JointType::Ball) ? 3 : (j.type == physics::JointType::Revolute ? 1 : 0);
    const DiffModel md = articulationToDiffModel(def, DiffContact::All);
    std::printf("amp_topology: bodies=%zu joints=%zu actuatedDOF=%d ndofJoints=%d floating=%d\n",
                def.bodies.size(), def.joints.size(), dof, md.ndofJoints, md.floating);
    TST_REQUIRE(dof == 28);
    TST_REQUIRE(md.links.size() == 15 && md.ndofJoints == 28 && md.floating);
}

// (2) Passive drop stays finite + rests on the plane (shape-aware all-body contact, shipped defaults).
TST_CASE(physics, integration, amp_humanoid_rests_on_ground) {
    const DiffModel md = articulationToDiffModel(physics::makeAMPHumanoid(), DiffContact::All);
    DiffState<double> st = makeState<double>(md); st.basePos = { 0, 1.10, 0 };   // drop from a bit above stand
    const V3<double> grav{ 0, -9.81, 0 }; const int substeps = 64; const double h = (1.0 / 60.0) / substeps;
    const std::vector<double> tau(static_cast<size_t>(md.ndofJoints), 0.0);
    double minSettledY = 1e9; bool finite = true;
    for (int c = 0; c < 300 && finite; ++c) {
        for (int i = 0; i < substeps; ++i) diffSubstep(md, st, tau, grav, h);
        const auto lw = linkWorld<double>(md, st);
        for (const auto& w : lw) finite = finite && std::isfinite(w.pos.y);
        if (c >= 240) for (const auto& w : lw) minSettledY = std::min(minSettledY, (double)w.pos.y);
    }
    const double pelvisY = linkWorld<double>(md, st)[0].pos.y;
    std::printf("amp_rests_on_ground: finite=%d settled pelvisY=%.4f minBodyY(last 1s)=%.4f\n", finite, pelvisY, minSettledY);
    TST_REQUIRE(finite);
    TST_REQUIRE(minSettledY > -0.03);    // rests ON the plane (no phase-through)
    TST_REQUIRE(pelvisY < 0.7);          // collapsed (passive), didn't magically stand
}
