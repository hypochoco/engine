//
//  articulation.cpp
//  engine::physics / dynamics
//

#include "engine/physics/dynamics/articulation.h"

namespace engine::physics {

Articulation buildArticulation(PhysicsWorld& world, const ArticulationDef& def) {
    Articulation out;
    out.bodies.reserve(def.bodies.size());
    for (const BodyDef& b : def.bodies) out.bodies.push_back(world.createBody(b));

    out.joints.reserve(def.joints.size());
    for (const JointSpec& s : def.joints) {
        JointDef jd;
        jd.type = s.type;
        jd.a = out.bodies[s.bodyA];
        jd.b = out.bodies[s.bodyB];
        jd.localAnchorA = s.localAnchorA;
        jd.localAnchorB = s.localAnchorB;
        jd.localAxisA = s.localAxisA;
        jd.localAxisB = s.localAxisB;
        jd.enableLimit = s.enableLimit;
        jd.lowerLimit = s.lowerLimit;
        jd.upperLimit = s.upperLimit;
        jd.actuator = s.actuator;
        out.joints.push_back(world.createJoint(jd));
    }
    return out;
}

namespace {

// Builder scratch: appends bodies/joints and wires anchors from world-space joint positions
// (bodies start axis-aligned at their world position, so local anchor = jointPos - bodyPos).
struct HumanoidBuilder {
    ArticulationDef def;
    uint32_t category;
    uint32_t mask;

    uint32_t addCapsule(Vec3 pos, Real radius, Real halfHeight, Real mass) {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.position = pos;
        d.mass = mass;
        d.collider.type = ColliderDesc::Type::Capsule;
        d.collider.capsule = Capsule{ radius, halfHeight };
        d.collisionCategory = category;
        d.collisionMask = mask;
        def.bodies.push_back(d);
        return static_cast<uint32_t>(def.bodies.size() - 1);
    }

    uint32_t addBox(Vec3 pos, Vec3 halfExtents, Real mass) {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.position = pos;
        d.mass = mass;
        d.collider.type = ColliderDesc::Type::Box;
        d.collider.box = Box{ halfExtents };
        d.collisionCategory = category;
        d.collisionMask = mask;
        def.bodies.push_back(d);
        return static_cast<uint32_t>(def.bodies.size() - 1);
    }

    void addJoint(JointType type, uint32_t parent, uint32_t child, Vec3 jointPos,
                  Vec3 axis = Vec3(1, 0, 0), bool limit = false, Real lo = 0, Real hi = 0) {
        JointSpec s;
        s.type = type;
        s.bodyA = parent;
        s.bodyB = child;
        s.localAnchorA = jointPos - def.bodies[parent].position;
        s.localAnchorB = jointPos - def.bodies[child].position;
        s.localAxisA = axis;
        s.localAxisB = axis;
        s.enableLimit = limit;
        s.lowerLimit = lo;
        s.upperLimit = hi;
        def.joints.push_back(s);
    }
};

} // namespace

ArticulationDef makeHumanoid(Vec3 root, uint32_t limbCategory) {
    HumanoidBuilder hb;
    hb.category = limbCategory;
    hb.mask = ~limbCategory;   // collide with everything except other limbs (no self-collision)
    const Real dy = root.y - 1.0f;   // vertical shift so `root` places the pelvis

    auto at = [&](float x, float y, float z) { return Vec3(root.x + x, y + dy, root.z + z); };

    // --- bodies (positions for a neutral standing pose) ---
    const uint32_t pelvis = hb.addBox(at(0.0f, 1.00f, 0.0f), Vec3(0.12f, 0.08f, 0.10f), 8.0f);
    const uint32_t torso  = hb.addBox(at(0.0f, 1.35f, 0.0f), Vec3(0.14f, 0.18f, 0.10f), 12.0f);
    const uint32_t head   = hb.addCapsule(at(0.0f, 1.62f, 0.0f), 0.09f, 0.05f, 4.0f);

    const uint32_t uArmL = hb.addCapsule(at( 0.30f, 1.42f, 0.0f), 0.05f, 0.13f, 2.0f);
    const uint32_t uArmR = hb.addCapsule(at(-0.30f, 1.42f, 0.0f), 0.05f, 0.13f, 2.0f);
    const uint32_t lArmL = hb.addCapsule(at( 0.30f, 1.13f, 0.0f), 0.045f, 0.12f, 1.5f);
    const uint32_t lArmR = hb.addCapsule(at(-0.30f, 1.13f, 0.0f), 0.045f, 0.12f, 1.5f);

    const uint32_t thighL = hb.addCapsule(at( 0.10f, 0.72f, 0.0f), 0.07f, 0.18f, 5.0f);
    const uint32_t thighR = hb.addCapsule(at(-0.10f, 0.72f, 0.0f), 0.07f, 0.18f, 5.0f);
    const uint32_t shinL  = hb.addCapsule(at( 0.10f, 0.34f, 0.0f), 0.05f, 0.18f, 3.0f);
    const uint32_t shinR  = hb.addCapsule(at(-0.10f, 0.34f, 0.0f), 0.05f, 0.18f, 3.0f);
    const uint32_t footL  = hb.addBox(at( 0.10f, 0.03f, 0.04f), Vec3(0.06f, 0.03f, 0.12f), 1.0f);
    const uint32_t footR  = hb.addBox(at(-0.10f, 0.03f, 0.04f), Vec3(0.06f, 0.03f, 0.12f), 1.0f);

    // --- joints (hinge axis X = flexion in the sagittal plane) ---
    hb.addJoint(JointType::Ball,     pelvis, torso, at(0.0f, 1.18f, 0.0f));            // waist
    hb.addJoint(JointType::Fixed,    torso,  head,  at(0.0f, 1.52f, 0.0f));            // neck
    hb.addJoint(JointType::Ball,     torso,  uArmL, at( 0.30f, 1.55f, 0.0f));          // shoulder L
    hb.addJoint(JointType::Ball,     torso,  uArmR, at(-0.30f, 1.55f, 0.0f));          // shoulder R
    hb.addJoint(JointType::Revolute, uArmL,  lArmL, at( 0.30f, 1.27f, 0.0f), Vec3(1,0,0), true, 0.0f, 2.5f);   // elbow L
    hb.addJoint(JointType::Revolute, uArmR,  lArmR, at(-0.30f, 1.27f, 0.0f), Vec3(1,0,0), true, 0.0f, 2.5f);   // elbow R
    hb.addJoint(JointType::Ball,     pelvis, thighL, at( 0.10f, 0.92f, 0.0f));         // hip L
    hb.addJoint(JointType::Ball,     pelvis, thighR, at(-0.10f, 0.92f, 0.0f));         // hip R
    hb.addJoint(JointType::Revolute, thighL, shinL, at( 0.10f, 0.54f, 0.0f), Vec3(1,0,0), true, -2.5f, 0.0f);  // knee L
    hb.addJoint(JointType::Revolute, thighR, shinR, at(-0.10f, 0.54f, 0.0f), Vec3(1,0,0), true, -2.5f, 0.0f);  // knee R
    hb.addJoint(JointType::Revolute, shinL,  footL, at( 0.10f, 0.10f, 0.0f), Vec3(1,0,0), true, -0.8f, 0.8f);  // ankle L
    hb.addJoint(JointType::Revolute, shinR,  footR, at(-0.10f, 0.10f, 0.0f), Vec3(1,0,0), true, -0.8f, 0.8f);  // ankle R

    return hb.def;
}

} // namespace engine::physics
