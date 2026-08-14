//
//  contact_quality.cpp
//  engine::tst / physics / integration
//
//  Investigates two reported realtime-solver quality issues (2026-08-01): (1) objects visibly
//  INTERPENETRATE before the solver corrects them, and (2) contact behaviour is "strange at angles."
//  Diagnostic (prints numbers; loose asserts) — characterizes the current SequentialImpulse+split-
//  impulse defaults; set PHYS_TGS=1 / PHYS_SUBSTEPS=n to compare the TGS path.
//

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/physics/physics.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
void applyEnv(WorldDef& wd) {
    if (const char* s = std::getenv("PHYS_TGS"))      wd.contactSolver =
        std::atoi(s) ? ContactSolver::TGSSoft : ContactSolver::SequentialImpulse;
    if (const char* s = std::getenv("PHYS_SUBSTEPS")) wd.substeps            = std::atoi(s);
    if (const char* s = std::getenv("PHYS_BAUM"))     wd.solver.contactBaumgarte = std::atof(s);
    if (const char* s = std::getenv("PHYS_SLOP"))     wd.solver.contactSlop      = std::atof(s);
}
WorldDef baseDef() {
    WorldDef wd;
    wd.gravity = Vec3(0, -18.0f, 0);
    wd.substeps = 4; wd.velocityIterations = 8;
    wd.linearDamping = 0.1f; wd.angularDamping = 0.2f;
    return wd;
}
} // namespace

// (1) INTERPENETRATION: a box dropped onto a static pedestal. Track how DEEP it penetrates on impact
// and how many frames until it's corrected back within ~1 mm. This is the "objects intersect a little
// before correcting" report — split-impulse depenetrates gradually (contactBaumgarte per substep).
TST_CASE(physics, integration, impact_penetration) {
    WorldDef wd = baseDef(); applyEnv(wd);
    auto w = createPhysicsWorld(Backend::Realtime, wd);

    BodyDef ped;
    ped.type = BodyType::Static;
    ped.collider.type = ColliderDesc::Type::Box;
    ped.collider.box = Box{ Vec3(2.0f, 0.5f, 2.0f) };
    ped.position = Vec3(0, 0.5f, 0);              // top at y = 1.0
    ped.material.friction = 0.8f;
    w->createBody(ped);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box;
    box.collider.box = Box{ Vec3(0.5f) };
    box.material.friction = 0.8f;
    box.position = Vec3(0, 2.3f, 0);              // drop from ~0.8 m up → ~5 m/s impact
    const BodyHandle bh = w->createBody(box);

    const float restY = 1.5f;                     // resting center height
    float maxPen = 0; int framesPenetrating = 0; int settleFrame = -1;
    for (int s = 0; s < 200; ++s) {
        w->step(1.0f / 60.0f);
        const float pen = restY - w->pose(bh).position.y;      // >0 = penetrating below rest
        if (pen > 0.001f) { maxPen = std::max(maxPen, pen); ++framesPenetrating; }
        if (settleFrame < 0 && s > 5 && std::abs(pen) < 0.006f &&
            glm::length(w->linearVelocities()[bh.index]) < 0.05f) settleFrame = s;
    }
    std::printf("impact_penetration: maxPen=%.4f m (%.1f mm)  framesPenetrating=%d  settleFrame=%d\n",
                maxPen, maxPen * 1000.0f, framesPenetrating, settleFrame);
    TST_REQUIRE_MSG(maxPen < 0.30f, "box tunnelled deeply into the pedestal");
}

// (2a) ANGLED LANDING: a box dropped tilted 25° must settle flat without excessive penetration of the
// leading corner, jitter, or drift. Characterizes "strange at angles" during the tip-to-rest.
TST_CASE(physics, integration, tilted_landing) {
    WorldDef wd = baseDef(); applyEnv(wd);
    auto w = createPhysicsWorld(Backend::Realtime, wd);

    BodyDef ground;
    ground.type = BodyType::Static;
    ground.collider.type = ColliderDesc::Type::Plane;
    ground.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
    ground.material.friction = 0.8f;
    w->createBody(ground);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box;
    box.collider.box = Box{ Vec3(0.5f) };
    box.material.friction = 0.8f;
    box.position = Vec3(0, 1.2f, 0);
    box.orientation = glm::angleAxis(glm::radians(25.0f), Vec3(0, 0, 1));   // tilted
    const BodyHandle bh = w->createBody(box);

    float maxLowCorner = 0;   // deepest penetration of any corner below ground
    for (int s = 0; s < 240; ++s) {
        w->step(1.0f / 60.0f);
        const engine::Transform p = w->pose(bh);
        for (int c = 0; c < 8; ++c) {
            const Vec3 local(((c&1)?0.5f:-0.5f), ((c&2)?0.5f:-0.5f), ((c&4)?0.5f:-0.5f));
            const float y = (p.position + p.rotation * local).y;
            if (y < 0) maxLowCorner = std::max(maxLowCorner, -y);
        }
    }
    const engine::Transform end = w->pose(bh);
    const float tiltDeg = 2 * std::acos(std::min(1.0f, std::abs(end.rotation.w))) * 57.2958f;
    const float drift = glm::length(glm::vec2(end.position.x, end.position.z));
    const float endOmega = glm::length(w->angularVelocities()[bh.index]);
    std::printf("tilted_landing: maxCornerPen=%.4f m (%.1f mm)  endTilt=%.1f deg  drift=%.3f  endOmega=%.3f\n",
                maxLowCorner, maxLowCorner * 1000.0f, tiltDeg, drift, endOmega);
    TST_REQUIRE_MSG(endOmega < 0.5f, "box still spinning after landing (angled instability)");
}

// (2b) INCLINE FRICTION: a box on a 15° ramp (well below the ~39° friction angle for mu=0.8) must
// STICK — and not drift SIDEWAYS. Sideways drift on a straight down-slope pull exposes the single-axis
// friction artifact (friction applied along the instantaneous slip direction, not a stable basis).
TST_CASE(physics, integration, incline_friction_stick) {
    WorldDef wd = baseDef(); applyEnv(wd);
    auto w = createPhysicsWorld(Backend::Realtime, wd);

    const float ang = glm::radians(15.0f);
    BodyDef ramp;
    ramp.type = BodyType::Static;
    ramp.collider.type = ColliderDesc::Type::Plane;
    ramp.collider.plane = Plane{ glm::normalize(Vec3(std::sin(ang), std::cos(ang), 0.0f)), 0.0f };  // tilt about Z
    ramp.material.friction = 0.8f;
    w->createBody(ramp);

    BodyDef box;
    box.type = BodyType::Dynamic; box.mass = 1.0f;
    box.collider.type = ColliderDesc::Type::Box;
    box.collider.box = Box{ Vec3(0.5f) };
    box.material.friction = 0.8f;
    box.orientation = glm::angleAxis(ang, Vec3(0, 0, 1));            // lie flat on the ramp
    box.position = Vec3(0, 0.55f, 0);
    const BodyHandle bh = w->createBody(box);

    for (int s = 0; s < 120; ++s) w->step(1.0f / 60.0f);            // settle onto the ramp
    const Vec3 p0 = w->pose(bh).position;
    for (int s = 0; s < 300; ++s) w->step(1.0f / 60.0f);            // hold (5 s)
    const Vec3 p1 = w->pose(bh).position;
    const float downSlope = glm::length(glm::vec2(p1.x - p0.x, p1.y - p0.y));  // in-plane (x,y) slide
    const float sideways  = std::abs(p1.z - p0.z);                            // out-of-plane drift
    std::printf("incline_friction: downSlopeSlide=%.4f  sidewaysDrift=%.4f  (15deg ramp, mu=0.8 → should stick)\n",
                downSlope, sideways);
    TST_REQUIRE_MSG(sideways < 0.05f, "box drifted sideways on the ramp (single-axis friction artifact)");
}
