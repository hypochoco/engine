//
//  from_articulation.h
//  engine::physics::diff
//
//  Convert a plain-data `physics::ArticulationDef` (e.g. `makeHumanoid()`) into a differentiable
//  `DiffModel` (Phase F3c), so the real articulation runs through the differentiable ABA. Mirrors
//  the reduced backend's model setup (featherstone_world.cpp): inertia from the collider, parent /
//  DOF / anchors / rest-relative rotation / hinge axis from the joints, floating root if the base
//  body is Dynamic. Assumes each joint's rest child-in-parent orientation is derived from the built
//  body orientations and (for revolute) the axis is given in a rest-identity-compatible frame — true
//  for `makeHumanoid` (all bodies axis-aligned). See notes/investigations/2026-07-04-differentiable-reduced.md.
//

#pragma once

#include <glm/gtc/quaternion.hpp>

#include "engine/physics/diff/articulated.h"
#include "engine/physics/dynamics/articulation.h"

namespace engine::physics::diff {

inline M3<double> glmToM3(const glm::mat3& g) {   // glm col-major → math (row,col)
    M3<double> r; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = static_cast<double>(g[j][i]); return r;
}

// Diagonal COM inertia matching featherstone_world.cpp::colliderInertia.
inline M3<double> colliderInertiaDiff(const physics::ColliderDesc& c, double m) {
    using T = physics::ColliderDesc::Type;
    switch (c.type) {
        case T::Sphere: { const double i = 0.4 * m * c.sphere.radius * c.sphere.radius; return diagM3(i, i, i); }
        case T::Box: {
            const double x2 = c.box.halfExtents.x * c.box.halfExtents.x, y2 = c.box.halfExtents.y * c.box.halfExtents.y, z2 = c.box.halfExtents.z * c.box.halfExtents.z;
            return diagM3(m / 3.0 * (y2 + z2), m / 3.0 * (x2 + z2), m / 3.0 * (x2 + y2));
        }
        case T::Capsule: {
            const double r = c.capsule.radius, h = 2.0 * c.capsule.halfHeight;
            return diagM3((1.0 / 12.0) * m * (3 * r * r + h * h), 0.5 * m * r * r, (1.0 / 12.0) * m * (3 * r * r + h * h));
        }
        default: { const double i = 0.4 * m; return diagM3(i, i, i); }
    }
}

// `footContactRadius > 0` gives the last two bodies (feet, by makeHumanoid contract) a ground
// contact sphere. Returns the DiffModel; the floating root is the parent-less Dynamic body.
inline DiffModel articulationToDiffModel(const physics::ArticulationDef& def, double footContactRadius = 0.0) {
    DiffModel md;
    md.links.resize(def.bodies.size());
    for (size_t i = 0; i < def.bodies.size(); ++i) {
        const physics::BodyDef& b = def.bodies[i];
        DiffLink& L = md.links[i];
        L.mass = static_cast<double>(b.mass);
        L.Ic = colliderInertiaDiff(b.collider, L.mass);
        L.parent = -1; L.dof = 0; L.restRel = identity3<double>();
    }
    int q = 0;
    for (const physics::JointSpec& s : def.joints) {
        DiffLink& L = md.links[s.bodyB];
        L.parent = static_cast<int>(s.bodyA);
        L.dof = (s.type == physics::JointType::Ball) ? 3 : (s.type == physics::JointType::Revolute ? 1 : 0);
        L.qIndex = q; q += L.dof;
        L.anchorP = { s.localAnchorA.x, s.localAnchorA.y, s.localAnchorA.z };
        L.anchorC = { s.localAnchorB.x, s.localAnchorB.y, s.localAnchorB.z };
        const glm::mat3 Rwp = glm::mat3_cast(def.bodies[s.bodyA].orientation);
        const glm::mat3 Rwc = glm::mat3_cast(def.bodies[s.bodyB].orientation);
        L.restRel = glmToM3(glm::transpose(Rwp) * Rwc);
        if (L.dof == 1) { const glm::vec3 ax = glm::normalize(s.localAxisA); L.axes[0] = { ax.x, ax.y, ax.z }; }
        else if (L.dof == 3) { L.axes[0] = { 1, 0, 0 }; L.axes[1] = { 0, 1, 0 }; L.axes[2] = { 0, 0, 1 }; }
    }
    md.ndofJoints = q;

    int root = -1;
    for (size_t i = 0; i < md.links.size(); ++i) if (md.links[i].parent < 0) { root = static_cast<int>(i); break; }
    md.floating = (root >= 0 && def.bodies[static_cast<size_t>(root)].type == physics::BodyType::Dynamic);

    if (footContactRadius > 0.0 && def.bodies.size() >= 2) {
        md.contactGround = true;
        md.links[def.bodies.size() - 1].contactRadius = footContactRadius;
        md.links[def.bodies.size() - 2].contactRadius = footContactRadius;
    }
    return md;
}

// Index of the floating/base root body (parent-less), or 0.
inline int rootBodyIndex(const DiffModel& md) {
    for (size_t i = 0; i < md.links.size(); ++i) if (md.links[i].parent < 0) return static_cast<int>(i);
    return 0;
}

} // namespace engine::physics::diff
