//
//  geo.cpp
//  engine::tst — geometry processing (engine::geo)
//
//  Verifies the capsule-SDF mesher produces watertight (no boundary edges) meshes with outward
//  normals, unions overlapping capsules into a single connected surface, and that QEM simplify
//  reduces the triangle budget while keeping the mesh closed and roughly shape-preserving.
//

#include "engine/core/geometry/geo.h"

#include "harness/harness.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <vector>

using namespace engine;

namespace {

// Count how many triangles reference each undirected edge. count==1 ⇒ boundary (hole); the mesher
// must produce closed surfaces (count>=2 everywhere), which is the "no gaps" property we're after.
int boundaryEdges(const MeshData& m) {
    std::map<std::pair<uint32_t, uint32_t>, int> edge;
    for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const uint32_t v[3] = { m.indices[t], m.indices[t + 1], m.indices[t + 2] };
        for (int e = 0; e < 3; ++e) {
            uint32_t a = v[e], b = v[(e + 1) % 3];
            if (a > b) std::swap(a, b);
            edge[{ a, b }]++;
        }
    }
    int boundary = 0;
    for (auto& [k, c] : edge) if (c == 1) ++boundary;
    return boundary;
}

// Fraction of triangles whose winding normal agrees with the (outward) vertex normals.
double outwardFraction(const MeshData& m) {
    if (m.indices.empty()) return 0.0;
    int ok = 0, total = 0;
    for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const auto& a = m.vertices[m.indices[t]];
        const auto& b = m.vertices[m.indices[t + 1]];
        const auto& c = m.vertices[m.indices[t + 2]];
        const glm::vec3 gn = glm::cross(b.position - a.position, c.position - a.position);
        const glm::vec3 vn = a.normal + b.normal + c.normal;
        if (glm::dot(gn, vn) > 0.0f) ++ok;
        ++total;
    }
    return double(ok) / total;
}

} // namespace

TST_CASE(core, unit, geo_mesher_sphere) {
    // A single "capsule" with a==b, ra==rb is a sphere of radius 1.
    std::vector<geo::Capsule> caps = { { glm::vec3(0.0f), glm::vec3(0.0f), 1.0f, 1.0f } };
    geo::MesherParams p; p.voxel = 0.12f;
    const MeshData m = geo::meshCapsules(caps, p);

    std::cout << "geo sphere: " << m.vertices.size() << " verts, " << m.indices.size() / 3 << " tris\n";
    TST_REQUIRE(!m.vertices.empty());
    TST_REQUIRE(m.indices.size() % 3 == 0);
    TST_REQUIRE(m.indices.size() >= 3);

    // Watertight (no holes) — the whole point.
    TST_REQUIRE(boundaryEdges(m) == 0);

    // Surface sits near radius 1 (within a couple voxels), and normals point outward.
    for (const auto& v : m.vertices) {
        const float r = glm::length(v.position);
        TST_REQUIRE(r > 0.75f && r < 1.25f);
        TST_REQUIRE(glm::dot(v.normal, glm::normalize(v.position)) > 0.5f);
    }
    TST_REQUIRE(outwardFraction(m) > 0.95);
}

TST_CASE(core, unit, geo_mesher_union) {
    // Two overlapping round-cones sharing an endpoint → must fuse into ONE closed surface.
    std::vector<geo::Capsule> caps = {
        { glm::vec3(-1.0f, 0, 0), glm::vec3(0.0f), 0.5f, 0.4f },
        { glm::vec3(0.0f), glm::vec3(1.0f, 0.6f, 0), 0.4f, 0.2f },
    };
    geo::MesherParams p; p.voxel = 0.08f; p.smooth = 0.15f;
    const MeshData m = geo::meshCapsules(caps, p);

    std::cout << "geo union: " << m.vertices.size() << " verts, " << m.indices.size() / 3 << " tris\n";
    TST_REQUIRE(!m.vertices.empty());
    TST_REQUIRE(boundaryEdges(m) == 0);            // fused + watertight

    // Single connected component (union-find over triangle vertices).
    std::vector<int> parent(m.vertices.size());
    for (size_t i = 0; i < parent.size(); ++i) parent[i] = int(i);
    std::function<int(int)> find = [&](int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; };
    auto uni = [&](int a, int b) { parent[find(a)] = find(b); };
    for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        uni(int(m.indices[t]), int(m.indices[t + 1]));
        uni(int(m.indices[t + 1]), int(m.indices[t + 2]));
    }
    int roots = 0;
    for (size_t i = 0; i < parent.size(); ++i) if (find(int(i)) == int(i)) ++roots;
    std::cout << "geo union components: " << roots << "\n";
    TST_REQUIRE(roots == 1);
}

TST_CASE(core, unit, geo_simplify) {
    std::vector<geo::Capsule> caps = { { glm::vec3(0.0f), glm::vec3(0.0f), 1.0f, 1.0f } };
    geo::MesherParams p; p.voxel = 0.1f;
    const MeshData full = geo::meshCapsules(caps, p);
    const size_t fullTris = full.indices.size() / 3;
    TST_REQUIRE(fullTris > 200);

    const size_t target = fullTris / 4;
    const MeshData simp = geo::simplify(full, target);
    const size_t simpTris = simp.indices.size() / 3;
    std::cout << "geo simplify: " << fullTris << " -> " << simpTris << " tris (target " << target << ")\n";

    TST_REQUIRE(simpTris < fullTris);              // actually reduced
    TST_REQUIRE(simpTris <= target + target / 5);  // reached ~target (within 20%)
    TST_REQUIRE(!simp.vertices.empty());
    TST_REQUIRE(boundaryEdges(simp) == 0);         // still closed

    // Shape preserved: still ~unit sphere.
    for (const auto& v : simp.vertices) {
        const float r = glm::length(v.position);
        TST_REQUIRE(r > 0.7f && r < 1.3f);
    }
    std::cout << "geo ok\n";
}
