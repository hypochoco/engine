//
//  primitives.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/primitives.h"

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

} // namespace engine::physics::collide
