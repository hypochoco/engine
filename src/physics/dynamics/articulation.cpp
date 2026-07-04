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

    uint32_t addSphere(Vec3 pos, Real radius, Real mass) {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.position = pos;
        d.mass = mass;
        d.collider.type = ColliderDesc::Type::Sphere;
        d.collider.sphere = Sphere{ radius };
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
    // Anthropometric layout for a 1.70 m figure (foot sole at model-y 0, head top at 1.70).
    // Segments do NOT overlap — each joint sits in the GAP between two bodies, so the limbs are
    // connected only by the (invisible) joint constraints. `rootPosition` places the pelvis;
    // model-space pelvis height is 0.99, so root=(0,0.99,0) stands the figure on y=0.
    const Real dy = root.y - Real(0.99);
    auto at = [&](float x, float y, float z) { return Vec3(root.x + x, y + dy, root.z + z); };

    // --- bodies (ellipsoid/capsule limbs; boxes for pelvis/torso/shoulders/head/feet) ---
    // FEET ARE THE LAST TWO BODIES by contract (the renderer draws the last two as boxes and all
    // others as stretched spheres).
    const uint32_t pelvis    = hb.addBox(at(0.0f, 0.99f, 0.0f), Vec3(0.10f, 0.08f, 0.08f), 10.0f);
    const uint32_t torso     = hb.addBox(at(0.0f, 1.26f, 0.0f), Vec3(0.13f, 0.15f, 0.09f), 14.0f);
    const uint32_t shoulders = hb.addBox(at(0.0f, 1.47f, 0.0f), Vec3(0.18f, 0.05f, 0.08f), 4.0f);
    const uint32_t head      = hb.addBox(at(0.0f, 1.615f, 0.0f), Vec3(0.075f, 0.085f, 0.08f), 4.5f);

    const uint32_t uArmL = hb.addCapsule(at( 0.23f, 1.30f, 0.0f), 0.045f, 0.105f, 2.0f);
    const uint32_t uArmR = hb.addCapsule(at(-0.23f, 1.30f, 0.0f), 0.045f, 0.105f, 2.0f);
    const uint32_t lArmL = hb.addCapsule(at( 0.23f, 0.98f, 0.0f), 0.040f, 0.090f, 1.4f);
    const uint32_t lArmR = hb.addCapsule(at(-0.23f, 0.98f, 0.0f), 0.040f, 0.090f, 1.4f);

    const uint32_t thighL = hb.addCapsule(at( 0.09f, 0.71f, 0.0f), 0.070f, 0.120f, 7.0f);
    const uint32_t thighR = hb.addCapsule(at(-0.09f, 0.71f, 0.0f), 0.070f, 0.120f, 7.0f);
    const uint32_t shinL  = hb.addCapsule(at( 0.09f, 0.29f, 0.0f), 0.050f, 0.140f, 3.2f);
    const uint32_t shinR  = hb.addCapsule(at(-0.09f, 0.29f, 0.0f), 0.050f, 0.140f, 3.2f);
    const uint32_t footL  = hb.addBox(at( 0.09f, 0.03f, 0.04f), Vec3(0.05f, 0.03f, 0.12f), 1.0f);
    const uint32_t footR  = hb.addBox(at(-0.09f, 0.03f, 0.04f), Vec3(0.05f, 0.03f, 0.12f), 1.0f);

    // --- joints (each anchored in the gap between its two bodies; hinge axis X = sagittal) ---
    hb.addJoint(JointType::Ball,     pelvis,    torso,     at(0.0f, 1.09f, 0.0f));    // 0 waist
    hb.addJoint(JointType::Fixed,    torso,     shoulders, at(0.0f, 1.42f, 0.0f));    // 1 chest
    hb.addJoint(JointType::Fixed,    shoulders, head,      at(0.0f, 1.525f, 0.0f));   // 2 neck
    hb.addJoint(JointType::Ball,     shoulders, uArmL,     at( 0.20f, 1.45f, 0.0f));  // 3 shoulder L
    hb.addJoint(JointType::Ball,     shoulders, uArmR,     at(-0.20f, 1.45f, 0.0f));  // 4 shoulder R
    hb.addJoint(JointType::Revolute, uArmL, lArmL, at( 0.23f, 1.13f, 0.0f), Vec3(1,0,0), true, 0.0f, 2.5f);   // 5 elbow L
    hb.addJoint(JointType::Revolute, uArmR, lArmR, at(-0.23f, 1.13f, 0.0f), Vec3(1,0,0), true, 0.0f, 2.5f);   // 6 elbow R
    hb.addJoint(JointType::Ball,     pelvis, thighL, at( 0.09f, 0.91f, 0.0f));        // 7 hip L
    hb.addJoint(JointType::Ball,     pelvis, thighR, at(-0.09f, 0.91f, 0.0f));        // 8 hip R
    hb.addJoint(JointType::Revolute, thighL, shinL, at( 0.09f, 0.50f, 0.0f), Vec3(1,0,0), true, -2.5f, 0.0f); // 9 knee L
    hb.addJoint(JointType::Revolute, thighR, shinR, at(-0.09f, 0.50f, 0.0f), Vec3(1,0,0), true, -2.5f, 0.0f); // 10 knee R
    hb.addJoint(JointType::Revolute, shinL, footL, at( 0.09f, 0.06f, 0.0f), Vec3(1,0,0), true, -0.8f, 0.8f);  // 11 ankle L
    hb.addJoint(JointType::Revolute, shinR, footR, at(-0.09f, 0.06f, 0.0f), Vec3(1,0,0), true, -0.8f, 0.8f);  // 12 ankle R

    return hb.def;
}

ArticulationDef makeAMPHumanoid(Vec3 root, uint32_t limbCategory) {
    HumanoidBuilder hb;
    hb.category = limbCategory;
    hb.mask = ~limbCategory;
    // DeepMimic/AMP topology (15 bodies, 28 actuated DOF), authored in our Y-up convention from
    // ase/data/assets/mjcf/amp_humanoid.xml (Z-up→Y-up map: ours = (amp.y, amp.z, amp.x); sole at y≈0).
    // Masses are approximate (the MJCF is density-based; faithful mass-from-density is deferred — see
    // notes/investigations/2026-07-04-humanoid-rig-adoption.md). Segments don't overlap; joints sit in
    // the gaps. FEET ARE THE LAST TWO BODIES by contract. Pelvis COM model-y = 1.022 ⇒ root places it.
    const Real dy = root.y - Real(1.022);
    auto at = [&](float x, float y, float z) { return Vec3(root.x + x, y + dy, root.z + z); };

    // --- bodies (spheres for pelvis/torso/head/hands, capsules for limbs, boxes for feet) ---
    const uint32_t pelvis = hb.addSphere (at(0.0f,     1.022f,  0.0f),           0.12f,             10.0f);
    const uint32_t torso  = hb.addSphere (at(0.0f,     1.238f,  0.0f),           0.11f,             14.0f);
    const uint32_t head   = hb.addSphere (at(0.0f,     1.517f,  0.0f),           0.095f,             4.5f);
    const uint32_t uArmR  = hb.addCapsule(at(-0.183f,  1.222f, -0.024f),  0.045f, 0.090f,            2.0f);
    const uint32_t lArmR  = hb.addCapsule(at(-0.183f,  0.967f, -0.024f),  0.040f, 0.0675f,           1.4f);
    const uint32_t handR  = hb.addSphere (at(-0.183f,  0.828f, -0.024f),          0.040f,            0.5f);
    const uint32_t uArmL  = hb.addCapsule(at( 0.183f,  1.222f, -0.024f),  0.045f, 0.090f,            2.0f);
    const uint32_t lArmL  = hb.addCapsule(at( 0.183f,  0.967f, -0.024f),  0.040f, 0.0675f,           1.4f);
    const uint32_t handL  = hb.addSphere (at( 0.183f,  0.828f, -0.024f),          0.040f,            0.5f);
    const uint32_t thighR = hb.addCapsule(at(-0.085f,  0.672f,  0.0f),    0.055f, 0.150f,            7.0f);
    const uint32_t shinR  = hb.addCapsule(at(-0.085f,  0.260f,  0.0f),    0.050f, 0.155f,            3.2f);
    const uint32_t thighL = hb.addCapsule(at( 0.085f,  0.672f,  0.0f),    0.055f, 0.150f,            7.0f);
    const uint32_t shinL  = hb.addCapsule(at( 0.085f,  0.260f,  0.0f),    0.050f, 0.155f,            3.2f);
    const uint32_t footR  = hb.addBox    (at(-0.085f,  0.0275f, 0.045f),  Vec3(0.045f, 0.0275f, 0.0885f), 1.0f);
    const uint32_t footL  = hb.addBox    (at( 0.085f,  0.0275f, 0.045f),  Vec3(0.045f, 0.0275f, 0.0885f), 1.0f);

    // --- joints (28 DOF): ball abdomen/neck/shoulders/hips/ankles, hinge elbows/knees, fixed wrists ---
    hb.addJoint(JointType::Ball,     pelvis, torso,  at(0.0f,    1.118f,  0.0f));                                    // abdomen (3)
    hb.addJoint(JointType::Ball,     torso,  head,   at(0.0f,    1.342f,  0.0f));                                    // neck (3)
    hb.addJoint(JointType::Ball,     torso,  uArmR,  at(-0.183f, 1.362f, -0.024f));                                  // shoulder R (3)
    hb.addJoint(JointType::Revolute, uArmR,  lArmR,  at(-0.183f, 1.087f, -0.024f), Vec3(1,0,0), true, 0.0f,  2.6f);  // elbow R (1)
    hb.addJoint(JointType::Fixed,    lArmR,  handR,  at(-0.183f, 0.828f, -0.024f));                                  // wrist R (0)
    hb.addJoint(JointType::Ball,     torso,  uArmL,  at( 0.183f, 1.362f, -0.024f));                                  // shoulder L (3)
    hb.addJoint(JointType::Revolute, uArmL,  lArmL,  at( 0.183f, 1.087f, -0.024f), Vec3(1,0,0), true, 0.0f,  2.6f);  // elbow L (1)
    hb.addJoint(JointType::Fixed,    lArmL,  handL,  at( 0.183f, 0.828f, -0.024f));                                  // wrist L (0)
    hb.addJoint(JointType::Ball,     pelvis, thighR, at(-0.085f, 0.882f,  0.0f));                                    // hip R (3)
    hb.addJoint(JointType::Revolute, thighR, shinR,  at(-0.085f, 0.460f,  0.0f),   Vec3(1,0,0), true, -2.6f, 0.0f);  // knee R (1)
    hb.addJoint(JointType::Ball,     shinR,  footR,  at(-0.085f, 0.050f,  0.0f));                                    // ankle R (3)
    hb.addJoint(JointType::Ball,     pelvis, thighL, at( 0.085f, 0.882f,  0.0f));                                    // hip L (3)
    hb.addJoint(JointType::Revolute, thighL, shinL,  at( 0.085f, 0.460f,  0.0f),   Vec3(1,0,0), true, -2.6f, 0.0f);  // knee L (1)
    hb.addJoint(JointType::Ball,     shinL,  footL,  at( 0.085f, 0.050f,  0.0f));                                    // ankle L (3)

    return hb.def;
}

} // namespace engine::physics
