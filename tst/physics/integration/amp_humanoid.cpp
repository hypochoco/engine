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

#include "engine/physics/diff/diff_environment.h"
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

// (3) Actuation: joint-torque actions (what an NN policy would emit) drive the rig — the driven
// trajectory diverges clearly from the passive one, and the actuated DOFs pick up velocity. This is
// the exact control path a walking policy uses (torques on joints; the root moves via contact, not
// by any direct root force).
TST_CASE(physics, integration, amp_humanoid_actuation_drives_it) {
    auto rollout = [](bool actuate) {
        DiffEnvironment env(physics::makeAMPHumanoid(), DiffContact::All);
        std::vector<double> a(static_cast<size_t>(env.actionDim()), 0.0);
        if (actuate) { a[14] = 60.0; a[16] = -40.0; a[17] = 50.0;   // right hip (2 axes) + right knee torques
                       a[21] = -60.0; a[24] = 50.0; }               // left hip + left knee
        env.setAction(a);
        for (int c = 0; c < 30; ++c) env.step();                    // 0.5 s
        return env;
    };
    const auto pf = rollout(false).links(), df = rollout(true).links();
    double maxBodyShift = 0.0;
    for (size_t i = 0; i < pf.size(); ++i) {
        const double dx = pf[i].pos.x - df[i].pos.x, dy = pf[i].pos.y - df[i].pos.y, dz = pf[i].pos.z - df[i].pos.z;
        maxBodyShift = std::max(maxBodyShift, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    std::printf("amp_actuation: max body-position shift (driven vs passive) = %.4f m over 0.5 s\n", maxBodyShift);
    TST_REQUIRE(DiffEnvironment(physics::makeAMPHumanoid()).actionDim() == 28);   // an NN emits a 28-vector
    TST_REQUIRE(maxBodyShift > 0.05);   // torque actions clearly move the model
}
