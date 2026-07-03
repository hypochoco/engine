//
//  primitives.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/primitives.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/quaternion.hpp>

namespace engine::physics::collide {

bool sphereVsPlane(const Vec3& center, const Sphere& sphere,
                   const Plane& plane, Real margin, Contact& out) {
    const Real dist = glm::dot(plane.normal, center) - plane.offset;  // center to surface
    out.normal = plane.normal;
    out.separation = dist - sphere.radius;                            // <0 => penetrating
    out.point = center - plane.normal * sphere.radius;                // sphere's lowest point
    out.touching = out.separation <= margin;
    return out.touching;
}

bool sphereVsSphere(const Vec3& centerA, const Sphere& a,
                    const Vec3& centerB, const Sphere& b, Real margin, Contact& out) {
    const Vec3 d = centerB - centerA;
    const Real dist = glm::length(d);
    out.normal = (dist > kEpsilon) ? d / dist : Vec3(0, 1, 0);
    out.separation = dist - (a.radius + b.radius);
    out.point = centerA + out.normal * a.radius;                      // on A's surface
    out.touching = out.separation <= margin;
    return out.touching;
}

bool sphereVsBox(const Vec3& boxCenter, const Quat& boxOrient, const Box& box,
                 const Vec3& sphereCenter, const Sphere& sphere, Real margin, Contact& out) {
    const Vec3 he = box.halfExtents;
    const Vec3 local = glm::conjugate(boxOrient) * (sphereCenter - boxCenter);   // sphere in box space
    const Vec3 clamped(std::clamp(local.x, -he.x, he.x),
                       std::clamp(local.y, -he.y, he.y),
                       std::clamp(local.z, -he.z, he.z));
    const Vec3 diff = local - clamped;
    const Real d2 = glm::dot(diff, diff);

    Vec3 nLocal;
    Real dist;
    if (d2 > kEpsilon * kEpsilon) {
        dist = std::sqrt(d2);
        nLocal = diff / dist;                                     // box surface -> sphere center
    } else {
        // Sphere center inside the box: push out along the least-penetrated axis.
        const Vec3 pen = he - glm::abs(local);                    // distance to each face
        if (pen.x <= pen.y && pen.x <= pen.z)      { nLocal = Vec3(local.x < 0 ? -1 : 1, 0, 0); dist = -pen.x; }
        else if (pen.y <= pen.z)                   { nLocal = Vec3(0, local.y < 0 ? -1 : 1, 0); dist = -pen.y; }
        else                                       { nLocal = Vec3(0, 0, local.z < 0 ? -1 : 1); dist = -pen.z; }
    }

    out.normal = boxOrient * nLocal;                              // world; box -> sphere
    out.separation = dist - sphere.radius;
    out.point = boxCenter + boxOrient * clamped;                  // closest point on the box surface
    out.touching = out.separation <= margin;
    return out.touching;
}

int pointsVsPlane(const Vec3* worldPoints, int count, const Plane& plane, Real margin, Contact out[4]) {
    Contact cand[64];
    int n = 0;
    for (int i = 0; i < count && n < 64; ++i) {
        const Real sep = glm::dot(plane.normal, worldPoints[i]) - plane.offset;
        if (sep <= margin) {
            cand[n].normal = plane.normal;   // plane -> shape
            cand[n].point = worldPoints[i];
            cand[n].separation = sep;
            cand[n].touching = true;
            ++n;
        }
    }
    const int keep = std::min(n, 4);
    for (int i = 0; i < keep; ++i) {
        int best = i;
        for (int j = i + 1; j < n; ++j)
            if (cand[j].separation < cand[best].separation) best = j;
        std::swap(cand[i], cand[best]);
        out[i] = cand[i];
    }
    return keep;
}

int boxVsPlane(const Vec3& boxCenter, const Quat& boxOrient, const Box& box,
               const Plane& plane, Real margin, Contact out[4]) {
    const Vec3 he = box.halfExtents;
    Vec3 corners[8];
    int n = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                corners[n++] = boxCenter + boxOrient * Vec3(sx * he.x, sy * he.y, sz * he.z);
    return pointsVsPlane(corners, 8, plane, margin, out);
}

} // namespace engine::physics::collide
