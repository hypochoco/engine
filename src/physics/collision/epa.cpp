//
//  epa.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/epa.h"

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace engine::physics {
namespace {

struct Face {
    int  a, b, c;
    Vec3 normal;   // outward (points away from origin, which is inside the polytope)
    Real dist;     // distance from origin to the face plane
};

Face makeFace(const std::vector<Vec3>& v, int a, int b, int c) {
    Vec3 n = glm::cross(v[b] - v[a], v[c] - v[a]);
    const Real len = glm::length(n);
    n = (len > kEpsilon) ? n / len : Vec3(0, 1, 0);
    Real d = glm::dot(n, v[a]);
    if (d < 0) { n = -n; d = -d; std::swap(b, c); }   // orient outward (origin inside)
    return Face{ a, b, c, n, d };
}

// Add an edge to the silhouette list, cancelling it if the reverse edge is already present
// (that edge is shared by two removed faces → interior, not part of the horizon).
void addEdge(std::vector<std::pair<int, int>>& edges, int i, int j) {
    for (size_t k = 0; k < edges.size(); ++k)
        if (edges[k].first == j && edges[k].second == i) {
            edges[k] = edges.back();
            edges.pop_back();
            return;
        }
    edges.emplace_back(i, j);
}

Vec3 tripleCross(const Vec3& a, const Vec3& b, const Vec3& c) {
    return glm::cross(glm::cross(a, b), c);
}

// Build a non-degenerate tetrahedron on the Minkowski surface enclosing the origin, via
// spanning support queries. Robust for shapes with coplanar supports (boxes), where GJK's
// terminating simplex is often degenerate.
bool buildTetra(const SupportShape& a, const SupportShape& b, Vec3 v[4]) {
    // A generic (off-axis) seed avoids the degenerate case where an axis-aligned penetration
    // puts the origin exactly on a tetra edge/face (e.g. two axis-aligned boxes).
    const Vec3 seed(Real(0.577), Real(0.577), Real(0.577));
    v[0] = minkowskiSupport(a, b, seed);
    Vec3 d = -v[0];
    if (glm::dot(d, d) < kEpsilon) d = -seed;

    v[1] = minkowskiSupport(a, b, d);
    const Vec3 ab = v[1] - v[0];
    Vec3 dir = tripleCross(ab, -v[0], ab);            // perpendicular to ab, toward origin
    if (glm::dot(dir, dir) < kEpsilon) {
        dir = glm::cross(ab, Vec3(1, 0, 0));
        if (glm::dot(dir, dir) < kEpsilon) dir = glm::cross(ab, Vec3(0, 0, 1));
    }
    v[2] = minkowskiSupport(a, b, dir);

    Vec3 n = glm::cross(v[1] - v[0], v[2] - v[0]);
    if (glm::dot(n, -v[0]) < 0) n = -n;               // point toward the origin side
    v[3] = minkowskiSupport(a, b, n);
    if (glm::dot(n, v[3] - v[0]) < kEpsilon)          // coplanar → search the other side
        v[3] = minkowskiSupport(a, b, -n);

    // Reject a flat tetra.
    const Real vol = glm::dot(glm::cross(v[1] - v[0], v[2] - v[0]), v[3] - v[0]);
    return std::fabs(vol) > kEpsilon;
}

} // namespace

EpaResult epaPenetration(const SupportShape& a, const SupportShape& b, const Simplex& simplex) {
    (void)simplex;   // rebuilt robustly below (GJK's simplex is often degenerate for boxes)

    Vec3 t[4];
    if (!buildTetra(a, b, t)) return {};
    std::vector<Vec3> verts{ t[0], t[1], t[2], t[3] };
    std::vector<Face> faces{
        makeFace(verts, 0, 1, 2), makeFace(verts, 0, 1, 3),
        makeFace(verts, 0, 2, 3), makeFace(verts, 1, 2, 3),
    };

    for (int iter = 0; iter < 64; ++iter) {
        // closest face to the origin
        int closest = 0;
        for (size_t i = 1; i < faces.size(); ++i)
            if (faces[i].dist < faces[closest].dist) closest = static_cast<int>(i);

        const Vec3 n = faces[closest].normal;
        const Real d = faces[closest].dist;

        const Vec3 p = minkowskiSupport(a, b, n);
        const Real pd = glm::dot(p, n);
        if (pd - d < Real(1e-4)) return { n, d, true };   // converged to the surface

        // Remove every face the new point can see; collect the horizon edges.
        const int vi = static_cast<int>(verts.size());
        verts.push_back(p);
        std::vector<std::pair<int, int>> edges;
        for (size_t i = 0; i < faces.size();) {
            if (glm::dot(faces[i].normal, p - verts[faces[i].a]) > 0) {
                addEdge(edges, faces[i].a, faces[i].b);
                addEdge(edges, faces[i].b, faces[i].c);
                addEdge(edges, faces[i].c, faces[i].a);
                faces[i] = faces.back();
                faces.pop_back();
            } else {
                ++i;
            }
        }
        for (const auto& [e0, e1] : edges) faces.push_back(makeFace(verts, e0, e1, vi));
        if (faces.empty()) return {};
    }

    // Fallback: return the current closest face.
    int closest = 0;
    for (size_t i = 1; i < faces.size(); ++i)
        if (faces[i].dist < faces[closest].dist) closest = static_cast<int>(i);
    return { faces[closest].normal, faces[closest].dist, true };
}

} // namespace engine::physics
