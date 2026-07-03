//
//  capsule.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/capsule.h"

#include <algorithm>
#include <cmath>

#include "engine/physics/collision/gjk_distance.h"

namespace engine::physics::collide {
namespace {

Vec3 closestOnSegment(const Vec3& p, const Vec3& a, const Vec3& b) {
    const Vec3 ab = b - a;
    const Real t = glm::dot(p - a, ab) / std::max(glm::dot(ab, ab), kEpsilon);
    return a + ab * std::clamp(t, Real(0), Real(1));
}

// Closest points between segments [p1,q1] and [p2,q2] (Ericson, Real-Time Collision Detection).
void closestSegmentSegment(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2,
                           Vec3& c1, Vec3& c2) {
    const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const Real a = glm::dot(d1, d1), e = glm::dot(d2, d2), f = glm::dot(d2, r);
    Real s, t;
    if (a <= kEpsilon && e <= kEpsilon) { c1 = p1; c2 = p2; return; }
    if (a <= kEpsilon) { s = 0; t = std::clamp(f / e, Real(0), Real(1)); }
    else {
        const Real c = glm::dot(d1, r);
        if (e <= kEpsilon) { t = 0; s = std::clamp(-c / a, Real(0), Real(1)); }
        else {
            const Real bb = glm::dot(d1, d2);
            const Real denom = a * e - bb * bb;
            s = (denom > kEpsilon) ? std::clamp((bb * f - c * e) / denom, Real(0), Real(1)) : Real(0);
            t = (bb * s + f) / e;
            if (t < 0)      { t = 0; s = std::clamp(-c / a, Real(0), Real(1)); }
            else if (t > 1) { t = 1; s = std::clamp((bb - c) / a, Real(0), Real(1)); }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

bool sphereLike(const Vec3& pA, Real rA, const Vec3& pB, Real rB, Real margin, Contact& out) {
    const Vec3 d = pB - pA;
    const Real dist = glm::length(d);
    out.normal = (dist > kEpsilon) ? d / dist : Vec3(0, 1, 0);   // A -> B
    out.separation = dist - (rA + rB);
    out.point = pA + out.normal * rA;
    out.touching = out.separation <= margin;
    return out.touching;
}

} // namespace

bool capsuleVsSphere(const Vec3& capCenter, const Quat& capOrient, const Capsule& cap,
                     const Vec3& sphereCenter, const Sphere& sphere, Real margin, Contact& out) {
    Vec3 a, b;
    capsuleSegment(capCenter, capOrient, cap, a, b);
    const Vec3 cp = closestOnSegment(sphereCenter, a, b);
    return sphereLike(cp, cap.radius, sphereCenter, sphere.radius, margin, out);   // capsule -> sphere
}

bool capsuleVsCapsule(const Vec3& cA, const Quat& qA, const Capsule& a,
                      const Vec3& cB, const Quat& qB, const Capsule& b, Real margin, Contact& out) {
    Vec3 a0, a1, b0, b1;
    capsuleSegment(cA, qA, a, a0, a1);
    capsuleSegment(cB, qB, b, b0, b1);
    Vec3 c1, c2;
    closestSegmentSegment(a0, a1, b0, b1, c1, c2);
    return sphereLike(c1, a.radius, c2, b.radius, margin, out);   // A -> B
}

int capsuleVsPlane(const Vec3& capCenter, const Quat& capOrient, const Capsule& cap,
                   const Plane& plane, Real margin, Contact out[2]) {
    Vec3 ends[2];
    capsuleSegment(capCenter, capOrient, cap, ends[0], ends[1]);
    int n = 0;
    for (const Vec3& e : ends) {
        const Real sep = glm::dot(plane.normal, e) - plane.offset - cap.radius;
        if (sep <= margin) {
            out[n].normal = plane.normal;                    // plane -> capsule
            out[n].point = e - plane.normal * cap.radius;    // point on the capsule surface
            out[n].separation = sep;
            out[n].touching = true;
            ++n;
        }
    }
    return n;
}

int capsuleVsConvex(const Vec3& capCenter, const Quat& capOrient, const Capsule& cap,
                    const SupportShape& convex, Real margin, Contact out[2]) {
    Vec3 a, b;
    capsuleSegment(capCenter, capOrient, cap, a, b);

    // Per-endpoint closest point to the convex → up to 2 contacts (stable when lying on a face).
    int cnt = 0;
    for (const Vec3& e : { a, b }) {
        const SupportShape point = SupportShape::sphere(e, 0);
        Vec3 we, wc;
        const Real d = gjkClosest(point, convex, we, wc);
        if (d > kEpsilon && d - cap.radius <= margin) {
            out[cnt].normal = (e - wc) / d;                  // convex -> capsule
            out[cnt].point = wc;
            out[cnt].separation = d - cap.radius;
            out[cnt].touching = true;
            ++cnt;
        }
    }
    if (cnt > 0) return cnt;

    // Otherwise use the segment's closest point (e.g. the cylinder side on an edge/corner).
    const SupportShape seg = SupportShape::capsule(capCenter, capOrient, 0, cap.halfHeight);
    Vec3 wSeg, wConvex;
    const Real d = gjkClosest(seg, convex, wSeg, wConvex);
    if (d > kEpsilon && d - cap.radius <= margin) {
        out[0].normal = (wSeg - wConvex) / d;                // convex -> capsule
        out[0].point = wConvex;
        out[0].separation = d - cap.radius;
        out[0].touching = true;
        return 1;
    }
    return 0;   // overlapping or separated
}

} // namespace engine::physics::collide
