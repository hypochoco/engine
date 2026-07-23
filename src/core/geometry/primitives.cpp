//
//  primitives.cpp
//  engine::core
//
//  Implementations of the procedural mesh generators.
//

#include "engine/core/geometry/primitives.h"

#include <algorithm>
#include <cmath>

namespace engine::primitives {

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

MeshData makeQuad() {
    MeshData m;
    m.vertices = {
        { {-0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}, {1.0f, 1.0f, 1.0f} },
        { { 0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}, {1.0f, 1.0f, 1.0f} },
        { { 0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f, 1.0f} },
        { {-0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}, {1.0f, 1.0f, 1.0f} },
    };
    m.indices = { 0, 1, 2, 2, 3, 0 };
    return m;
}

MeshData makePlane(float size, uint32_t subdivisions) {
    subdivisions = std::max(subdivisions, 1u);

    MeshData m;
    const uint32_t verticesPerSide = subdivisions + 1;
    const float    half = size * 0.5f;
    const float    step = size / static_cast<float>(subdivisions);
    const float    uvStep = 1.0f / static_cast<float>(subdivisions);

    m.vertices.reserve(static_cast<size_t>(verticesPerSide) * verticesPerSide);
    for (uint32_t z = 0; z < verticesPerSide; ++z) {
        for (uint32_t x = 0; x < verticesPerSide; ++x) {
            Vertex v;
            v.position = { -half + step * static_cast<float>(x),
                           0.0f,
                           -half + step * static_cast<float>(z) };
            v.normal   = { 0.0f, 1.0f, 0.0f };
            v.uv       = { uvStep * static_cast<float>(x), uvStep * static_cast<float>(z) };
            v.color    = { 1.0f, 1.0f, 1.0f };
            m.vertices.push_back(v);
        }
    }

    m.indices.reserve(static_cast<size_t>(subdivisions) * subdivisions * 6);
    for (uint32_t z = 0; z < subdivisions; ++z) {
        for (uint32_t x = 0; x < subdivisions; ++x) {
            const uint32_t i0 = z * verticesPerSide + x;
            const uint32_t i1 = i0 + 1;
            const uint32_t i2 = i0 + verticesPerSide;
            const uint32_t i3 = i2 + 1;
            // CCW when viewed from +Y
            m.indices.insert(m.indices.end(), { i0, i2, i1, i1, i2, i3 });
        }
    }
    return m;
}

MeshData makeSphere(float radius, uint32_t rings, uint32_t sectors) {
    rings   = std::max(rings, 2u);
    sectors = std::max(sectors, 3u);

    MeshData m;
    const uint32_t vertsPerRow = sectors + 1;

    m.vertices.reserve(static_cast<size_t>(rings + 1) * vertsPerRow);
    for (uint32_t r = 0; r <= rings; ++r) {
        const float v      = static_cast<float>(r) / static_cast<float>(rings); // 0..1
        const float phi    = v * kPi;                                           // latitude 0..PI
        const float cosPhi = std::cos(phi);
        const float sinPhi = std::sin(phi);
        for (uint32_t s = 0; s <= sectors; ++s) {
            const float u     = static_cast<float>(s) / static_cast<float>(sectors); // 0..1
            const float theta = u * 2.0f * kPi;                                      // longitude
            const glm::vec3 n = { sinPhi * std::cos(theta), cosPhi, sinPhi * std::sin(theta) };

            Vertex vert;
            vert.normal   = n;
            vert.position = radius * n;
            vert.uv       = { u, v };
            vert.color    = { 1.0f, 1.0f, 1.0f };
            m.vertices.push_back(vert);
        }
    }

    m.indices.reserve(static_cast<size_t>(rings) * sectors * 6);
    for (uint32_t r = 0; r < rings; ++r) {
        for (uint32_t s = 0; s < sectors; ++s) {
            const uint32_t i0 = r * vertsPerRow + s;
            const uint32_t i1 = i0 + 1;
            const uint32_t i2 = i0 + vertsPerRow;
            const uint32_t i3 = i2 + 1;
            // CCW around the outward (radial) normal — matches quad/plane/box. The (phi,theta)
            // parameterization has ∂phi×∂theta pointing inward, so we wind i0→i1→i2 (not i0→i2→i1).
            m.indices.insert(m.indices.end(), { i0, i1, i2, i1, i3, i2 });
        }
    }
    return m;
}

MeshData makeBox(glm::vec3 h) {
    MeshData m;
    // Six faces, each with its own outward normal (flat shading) → 24 vertices, 36 indices.
    struct Face { glm::vec3 n, u, v; };   // normal + the two in-plane axes (half-extent scaled)
    // Per-face (normal, u, v) with (u, v, n) right-handed so cross(u,v) == n ⇒ every face is wound
    // counter-clockwise around its outward normal (the engine convention; required for back-face cull).
    const Face faces[6] = {
        { { 1, 0, 0}, {0, 0, -1}, {0, 1, 0} },   // +X
        { {-1, 0, 0}, {0, 0,  1}, {0, 1, 0} },   // -X
        { { 0, 1, 0}, {0, 0,  1}, {1, 0, 0} },   // +Y
        { { 0,-1, 0}, {0, 0, -1}, {1, 0, 0} },   // -Y
        { { 0, 0, 1}, {1, 0,  0}, {0, 1, 0} },   // +Z
        { { 0, 0,-1}, {-1,0,  0}, {0, 1, 0} },   // -Z
    };
    for (const Face& f : faces) {
        const glm::vec3 c = f.n * h;                    // face center
        const glm::vec3 u = f.u * h;                    // half-edge along u
        const glm::vec3 v = f.v * h;                    // half-edge along v
        const auto base = static_cast<uint32_t>(m.vertices.size());
        const glm::vec3 corners[4] = { c - u - v, c + u - v, c + u + v, c - u + v };
        const glm::vec2 uvs[4] = { {0,0}, {1,0}, {1,1}, {0,1} };
        for (int i = 0; i < 4; ++i)
            m.vertices.push_back(Vertex{ corners[i], f.n, uvs[i], glm::vec3(1.0f) });
        m.indices.insert(m.indices.end(),
                         { base, base + 1, base + 2, base + 2, base + 3, base });
    }
    return m;
}

MeshData makeCapsule(float radius, float halfHeight, uint32_t rings, uint32_t sectors) {
    rings   = std::max(rings, 1u);
    sectors = std::max(sectors, 3u);

    MeshData m;
    const uint32_t vertsPerRow = sectors + 1;
    const uint32_t totalRows = 2 * (rings + 1);   // top hemi + bottom hemi (both include equator)
    m.vertices.reserve(static_cast<size_t>(totalRows) * vertsPerRow);

    // Emit one latitude row of vertices. `phi` is the polar angle (0 = +Y pole, PI = -Y pole) and
    // `yCenter` is the hemisphere's center (+halfHeight for the top cap, -halfHeight for bottom);
    // the cylinder falls out of connecting the two equator rows (both at radius, different y).
    auto addRow = [&](float phi, float yCenter, float v) {
        const float cosPhi = std::cos(phi), sinPhi = std::sin(phi);
        for (uint32_t s = 0; s <= sectors; ++s) {
            const float u     = static_cast<float>(s) / static_cast<float>(sectors);
            const float theta = u * 2.0f * kPi;
            const glm::vec3 n = { sinPhi * std::cos(theta), cosPhi, sinPhi * std::sin(theta) };
            Vertex vert;
            vert.normal   = n;                                  // radial from the cap center
            vert.position = { radius * n.x, yCenter + radius * n.y, radius * n.z };
            vert.uv       = { u, v };
            vert.color    = { 1.0f, 1.0f, 1.0f };
            m.vertices.push_back(vert);
        }
    };
    for (uint32_t i = 0; i <= rings; ++i) {   // top hemisphere: phi 0..PI/2, center +halfHeight
        const float t = static_cast<float>(i) / static_cast<float>(rings);
        addRow(t * (kPi * 0.5f), halfHeight, t * 0.5f);
    }
    for (uint32_t i = 0; i <= rings; ++i) {   // bottom hemisphere: phi PI/2..PI, center -halfHeight
        const float t = static_cast<float>(i) / static_cast<float>(rings);
        addRow(kPi * 0.5f + t * (kPi * 0.5f), -halfHeight, 0.5f + t * 0.5f);
    }

    m.indices.reserve(static_cast<size_t>(totalRows - 1) * sectors * 6);
    for (uint32_t r = 0; r < totalRows - 1; ++r) {
        for (uint32_t s = 0; s < sectors; ++s) {
            const uint32_t i0 = r * vertsPerRow + s;
            const uint32_t i1 = i0 + 1;
            const uint32_t i2 = i0 + vertsPerRow;
            const uint32_t i3 = i2 + 1;
            // CCW around the outward (radial) normal — matches quad/plane/box. The (phi,theta)
            // parameterization has ∂phi×∂theta pointing inward, so we wind i0→i1→i2 (not i0→i2→i1).
            m.indices.insert(m.indices.end(), { i0, i1, i2, i1, i3, i2 });
        }
    }
    return m;
}

} // namespace engine::primitives
