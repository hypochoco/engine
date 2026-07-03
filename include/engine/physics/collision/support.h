//
//  support.h
//  engine::physics / collision
//
//  A convex shape instance (shape + world pose) exposing a support function: the farthest
//  point of the shape along a world-space direction. This single function powers GJK/EPA for
//  any convex shape (sphere, box, and later capsule/convex-hull). The Minkowski-difference
//  support A⊖B(d) = A.support(d) − B.support(−d) is what GJK/EPA actually search.
//

#pragma once

#include <glm/gtc/quaternion.hpp>

#include "engine/physics/types.h"

namespace engine::physics {

struct SupportShape {
    enum class Kind { Sphere, Box, Hull, Capsule };

    Kind kind = Kind::Sphere;
    Vec3 center{0};
    Quat orient{1, 0, 0, 0};
    Real radius = Real(0.5);          // sphere / capsule
    Real halfHeight = Real(0.5);      // capsule
    Vec3 halfExtents{Real(0.5)};      // box
    const Vec3* verts = nullptr;      // hull (local-space vertices)
    int         vertCount = 0;

    static SupportShape sphere(const Vec3& c, Real r) {
        SupportShape s; s.kind = Kind::Sphere; s.center = c; s.radius = r; return s;
    }
    static SupportShape box(const Vec3& c, const Quat& q, const Vec3& he) {
        SupportShape s; s.kind = Kind::Box; s.center = c; s.orient = q; s.halfExtents = he; return s;
    }
    static SupportShape hull(const Vec3& c, const Quat& q, const Vec3* v, int n) {
        SupportShape s; s.kind = Kind::Hull; s.center = c; s.orient = q; s.verts = v; s.vertCount = n; return s;
    }
    static SupportShape capsule(const Vec3& c, const Quat& q, Real r, Real hh) {
        SupportShape s; s.kind = Kind::Capsule; s.center = c; s.orient = q; s.radius = r; s.halfHeight = hh; return s;
    }

    Vec3 support(const Vec3& dir) const {
        if (kind == Kind::Sphere) {
            const Real len = glm::length(dir);
            return (len > kEpsilon) ? center + dir * (radius / len) : center;
        }
        if (kind == Kind::Capsule) {
            const Vec3 axis = orient * Vec3(0, halfHeight, 0);
            const Vec3 p = (glm::dot(dir, axis) >= 0) ? center + axis : center - axis;
            const Real len = glm::length(dir);
            return (len > kEpsilon) ? p + dir * (radius / len) : p;
        }
        if (kind == Kind::Box) {
            const Vec3 d = glm::conjugate(orient) * dir;   // world dir -> box local
            const Vec3 corner(d.x >= 0 ? halfExtents.x : -halfExtents.x,
                              d.y >= 0 ? halfExtents.y : -halfExtents.y,
                              d.z >= 0 ? halfExtents.z : -halfExtents.z);
            return center + orient * corner;
        }
        // hull: farthest local vertex along the (local) direction
        const Vec3 d = glm::conjugate(orient) * dir;
        int best = 0;
        Real bd = glm::dot(verts[0], d);
        for (int i = 1; i < vertCount; ++i) {
            const Real dd = glm::dot(verts[i], d);
            if (dd > bd) { bd = dd; best = i; }
        }
        return center + orient * verts[best];
    }
};

inline Vec3 minkowskiSupport(const SupportShape& a, const SupportShape& b, const Vec3& d) {
    return a.support(d) - b.support(-d);
}

} // namespace engine::physics
