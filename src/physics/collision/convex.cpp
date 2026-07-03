//
//  convex.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/convex.h"

#include "engine/physics/collision/epa.h"
#include "engine/physics/collision/gjk.h"

namespace engine::physics::collide {

bool convexVsConvex(const SupportShape& a, const SupportShape& b, Contact& out) {
    Simplex simplex;
    if (!gjkIntersect(a, b, simplex)) return false;

    const EpaResult r = epaPenetration(a, b, simplex);
    if (!r.ok) return false;

    Vec3 n = r.normal;
    if (glm::dot(n, b.center - a.center) < 0) n = -n;   // orient A -> B

    out.normal = n;
    out.separation = -r.depth;
    // Approximate contact point: midpoint of the two witness points along the normal.
    const Vec3 pA = a.support(n);
    const Vec3 pB = b.support(-n);
    out.point = (pA + pB) * Real(0.5);
    out.touching = true;
    return true;
}

} // namespace engine::physics::collide
