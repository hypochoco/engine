//
//  gjk.h
//  engine::physics / collision
//
//  Gilbert–Johnson–Keerthi boolean intersection test for two convex shapes. On overlap it
//  returns the enclosing simplex (a tetrahedron) that EPA expands to recover penetration.
//

#pragma once

#include <algorithm>

#include "engine/physics/collision/support.h"

namespace engine::physics {

struct Simplex {
    Vec3 pts[4];
    int  count = 0;

    void set4(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        pts[0] = a; pts[1] = b; pts[2] = c; pts[3] = d; count = 4;
    }
    void set3(const Vec3& a, const Vec3& b, const Vec3& c) {
        pts[0] = a; pts[1] = b; pts[2] = c; count = 3;
    }
    void set2(const Vec3& a, const Vec3& b) { pts[0] = a; pts[1] = b; count = 2; }
    void push(const Vec3& p) {
        pts[3] = pts[2]; pts[2] = pts[1]; pts[1] = pts[0]; pts[0] = p;
        count = std::min(count + 1, 4);
    }
};

bool gjkIntersect(const SupportShape& a, const SupportShape& b, Simplex& out);

} // namespace engine::physics
