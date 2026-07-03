//
//  box_box.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/box_box.h"

#include <cmath>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "engine/physics/collision/convex.h"
#include "engine/physics/collision/support.h"

namespace engine::physics::collide {
namespace {

struct Face {
    Vec3 center;
    Vec3 normal;          // outward, unit
    Vec3 u, w;            // in-plane unit axes
    Real hu, hw;          // half-lengths along u, w
    Vec3 v[4];            // corner vertices (world)
};

// The face of a box whose outward normal is most aligned with `dir`.
Face bestFace(const Vec3& c, const Mat3& R, const Vec3& h, const Vec3& dir) {
    int axis = 0; Real sign = 1; Real best = -1e30f;
    for (int k = 0; k < 3; ++k)
        for (Real s : { Real(1), Real(-1) }) {
            const Real d = glm::dot(R[k] * s, dir);
            if (d > best) { best = d; axis = k; sign = s; }
        }
    const int i = (axis + 1) % 3, j = (axis + 2) % 3;
    Face f;
    f.normal = R[axis] * sign;
    f.center = c + f.normal * h[axis];
    f.u = R[i]; f.hu = h[i];
    f.w = R[j]; f.hw = h[j];
    f.v[0] = f.center + f.u * f.hu + f.w * f.hw;
    f.v[1] = f.center - f.u * f.hu + f.w * f.hw;
    f.v[2] = f.center - f.u * f.hu - f.w * f.hw;
    f.v[3] = f.center + f.u * f.hu - f.w * f.hw;
    return f;
}

Real faceAlign(const Vec3& c, const Mat3& R, const Vec3& h, const Vec3& dir) {
    (void)c; (void)h;
    Real best = -1e30f;
    for (int k = 0; k < 3; ++k) best = std::max(best, std::fabs(glm::dot(R[k], dir)));
    return best;
}

// Sutherland-Hodgman: keep the part of `poly` on the inside half-space dot(n,p) <= offset.
void clip(std::vector<Vec3>& poly, const Vec3& n, Real offset) {
    std::vector<Vec3> out;
    const size_t m = poly.size();
    for (size_t k = 0; k < m; ++k) {
        const Vec3& cur = poly[k];
        const Vec3& prev = poly[(k + m - 1) % m];
        const Real dc = glm::dot(n, cur) - offset;
        const Real dp = glm::dot(n, prev) - offset;
        const bool curIn = dc <= 0, prevIn = dp <= 0;
        if (curIn) {
            if (!prevIn) out.push_back(prev + (cur - prev) * (dp / (dp - dc)));
            out.push_back(cur);
        } else if (prevIn) {
            out.push_back(prev + (cur - prev) * (dp / (dp - dc)));
        }
    }
    poly.swap(out);
}

} // namespace

int boxVsBox(const Vec3& ca, const Quat& qa, const Box& a,
             const Vec3& cb, const Quat& qb, const Box& b, Contact out[4]) {
    const auto sa = SupportShape::box(ca, qa, a.halfExtents);
    const auto sb = SupportShape::box(cb, qb, b.halfExtents);
    Contact epa;
    if (!convexVsConvex(sa, sb, epa)) return 0;
    const Vec3 n = epa.normal;   // A -> B

    const Mat3 Ra = glm::mat3_cast(qa);
    const Mat3 Rb = glm::mat3_cast(qb);

    // Reference = box whose face is most parallel to the normal; incident = the other.
    const bool refIsA = faceAlign(ca, Ra, a.halfExtents, n) >= faceAlign(cb, Rb, b.halfExtents, n);
    const Face ref = refIsA ? bestFace(ca, Ra, a.halfExtents, n)
                            : bestFace(cb, Rb, b.halfExtents, -n);
    const Face inc = refIsA ? bestFace(cb, Rb, b.halfExtents, -ref.normal)
                            : bestFace(ca, Ra, a.halfExtents, -ref.normal);

    // Clip the incident face polygon against the reference face's 4 side planes.
    std::vector<Vec3> poly{ inc.v[0], inc.v[1], inc.v[2], inc.v[3] };
    clip(poly,  ref.u, glm::dot(ref.u, ref.center) + ref.hu);
    clip(poly, -ref.u, glm::dot(-ref.u, ref.center) + ref.hu);
    clip(poly,  ref.w, glm::dot(ref.w, ref.center) + ref.hw);
    clip(poly, -ref.w, glm::dot(-ref.w, ref.center) + ref.hw);

    // Keep points below the reference face; contact separation = signed distance below it.
    Contact cand[16];
    int n0 = 0;
    const Real refOffset = glm::dot(ref.normal, ref.center);
    for (const Vec3& p : poly) {
        const Real sep = glm::dot(ref.normal, p) - refOffset;
        if (sep <= 0 && n0 < 16) {
            cand[n0].normal = n;                 // A -> B
            cand[n0].point = p;
            cand[n0].separation = sep;
            cand[n0].touching = true;
            ++n0;
        }
    }

    if (n0 == 0) { out[0] = epa; return 1; }     // fallback: single EPA point

    // Keep up to 4 deepest.
    const int keep = std::min(n0, 4);
    for (int i = 0; i < keep; ++i) {
        int bi = i;
        for (int j = i + 1; j < n0; ++j)
            if (cand[j].separation < cand[bi].separation) bi = j;
        std::swap(cand[i], cand[bi]);
        out[i] = cand[i];
    }
    return keep;
}

} // namespace engine::physics::collide
