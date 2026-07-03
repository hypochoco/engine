//
//  gjk.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/gjk.h"

namespace engine::physics {
namespace {

bool sameDir(const Vec3& a, const Vec3& b) { return glm::dot(a, b) > Real(0); }

// Triple product (a×b)×c — direction toward the origin, coplanar with a,b.
Vec3 tripleCross(const Vec3& a, const Vec3& b, const Vec3& c) {
    return glm::cross(glm::cross(a, b), c);
}

bool line(Simplex& s, Vec3& dir) {
    const Vec3 A = s.pts[0], B = s.pts[1];
    const Vec3 AB = B - A, AO = -A;
    if (sameDir(AB, AO)) {
        dir = tripleCross(AB, AO, AB);
        if (glm::dot(dir, dir) < kEpsilon) {                 // origin on the line AB
            dir = glm::cross(AB, Vec3(1, 0, 0));
            if (glm::dot(dir, dir) < kEpsilon) dir = glm::cross(AB, Vec3(0, 1, 0));
        }
    } else {
        s.set2(A, A); s.count = 1; dir = AO;
    }
    return false;
}

bool triangle(Simplex& s, Vec3& dir);

bool triangle(Simplex& s, Vec3& dir) {
    const Vec3 A = s.pts[0], B = s.pts[1], C = s.pts[2];
    const Vec3 AB = B - A, AC = C - A, AO = -A;
    const Vec3 ABC = glm::cross(AB, AC);

    if (sameDir(glm::cross(ABC, AC), AO)) {
        if (sameDir(AC, AO)) {
            s.set2(A, C); dir = tripleCross(AC, AO, AC);
            return false;
        }
        s.set2(A, B); return line(s, dir);
    }
    if (sameDir(glm::cross(AB, ABC), AO)) {
        s.set2(A, B); return line(s, dir);
    }
    // origin is above/below the triangle face
    if (sameDir(ABC, AO)) { dir = ABC; }
    else { s.set3(A, C, B); dir = -ABC; }
    return false;
}

bool tetrahedron(Simplex& s, Vec3& dir) {
    const Vec3 A = s.pts[0], B = s.pts[1], C = s.pts[2], D = s.pts[3];
    const Vec3 AB = B - A, AC = C - A, AD = D - A, AO = -A;
    const Vec3 ABC = glm::cross(AB, AC);
    const Vec3 ACD = glm::cross(AC, AD);
    const Vec3 ADB = glm::cross(AD, AB);

    if (sameDir(ABC, AO)) { s.set3(A, B, C); return triangle(s, dir); }
    if (sameDir(ACD, AO)) { s.set3(A, C, D); return triangle(s, dir); }
    if (sameDir(ADB, AO)) { s.set3(A, D, B); return triangle(s, dir); }
    return true;   // origin enclosed
}

bool nextSimplex(Simplex& s, Vec3& dir) {
    switch (s.count) {
        case 2:  return line(s, dir);
        case 3:  return triangle(s, dir);
        case 4:  return tetrahedron(s, dir);
        default: return false;
    }
}

} // namespace

bool gjkIntersect(const SupportShape& a, const SupportShape& b, Simplex& out) {
    Vec3 dir = b.center - a.center;
    if (glm::dot(dir, dir) < kEpsilon) dir = Vec3(1, 0, 0);

    Simplex s;
    s.push(minkowskiSupport(a, b, dir));
    dir = -s.pts[0];

    for (int iter = 0; iter < 64; ++iter) {
        if (glm::dot(dir, dir) < kEpsilon) dir = Vec3(1, 0, 0);
        const Vec3 p = minkowskiSupport(a, b, dir);
        if (glm::dot(p, dir) < 0) return false;      // no overlap along the search direction
        s.push(p);
        if (nextSimplex(s, dir)) { out = s; return true; }
    }
    out = s;
    return true;
}

} // namespace engine::physics
