//
//  bounds.h
//  engine::core / math
//
//  Backend-agnostic bounding volumes + a view frustum, for visibility culling (and reusable by a
//  future BVH). glm-only, no renderer/ECS dependencies.
//

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <glm/glm.hpp>

namespace engine::core {

// Axis-aligned bounding box. `empty()` (min > max) is the identity for merges.
struct Aabb {
    glm::vec3 min{ std::numeric_limits<float>::max() };
    glm::vec3 max{ -std::numeric_limits<float>::max() };

    bool      empty()   const { return min.x > max.x || min.y > max.y || min.z > max.z; }
    glm::vec3 center()  const { return 0.5f * (min + max); }
    glm::vec3 extents() const { return 0.5f * (max - min); }   // half-sizes

    void expand(const glm::vec3& p) { min = glm::min(min, p); max = glm::max(max, p); }
    void merge(const Aabb& o)       { min = glm::min(min, o.min); max = glm::max(max, o.max); }

    // Tight AABB of this box transformed by `m` (center + abs(rotation/scale)·extent — no need to
    // transform all 8 corners). Translation from the matrix's 4th column.
    Aabb transformed(const glm::mat4& m) const {
        if (empty()) return *this;
        const glm::vec3 c = center();
        const glm::vec3 e = extents();
        const glm::vec3 nc = glm::vec3(m * glm::vec4(c, 1.0f));
        // abs of the upper-left 3x3 applied to the extents.
        const glm::mat3 r = glm::mat3(m);
        const glm::vec3 ne{
            std::abs(r[0].x) * e.x + std::abs(r[1].x) * e.y + std::abs(r[2].x) * e.z,
            std::abs(r[0].y) * e.x + std::abs(r[1].y) * e.y + std::abs(r[2].y) * e.z,
            std::abs(r[0].z) * e.x + std::abs(r[1].z) * e.y + std::abs(r[2].z) * e.z,
        };
        return Aabb{ nc - ne, nc + ne };
    }
};

// View frustum as 6 inward-pointing planes (x·n + d >= 0 inside). Extracted from a view-projection
// matrix via Gribb-Hartmann for a 0..1 clip-space depth range (GLM_FORCE_DEPTH_ZERO_TO_ONE, which
// the engine sets globally — Metal/Vulkan convention).
struct Frustum {
    // Each plane is (nx, ny, nz, d) with the normal pointing INTO the frustum.
    std::array<glm::vec4, 6> planes{};

    static Frustum fromViewProj(const glm::mat4& vp) {
        // Rows of vp (glm is column-major: vp[col][row]).
        const glm::vec4 r0{ vp[0][0], vp[1][0], vp[2][0], vp[3][0] };
        const glm::vec4 r1{ vp[0][1], vp[1][1], vp[2][1], vp[3][1] };
        const glm::vec4 r2{ vp[0][2], vp[1][2], vp[2][2], vp[3][2] };
        const glm::vec4 r3{ vp[0][3], vp[1][3], vp[2][3], vp[3][3] };
        Frustum f;
        f.planes[0] = r3 + r0;   // left
        f.planes[1] = r3 - r0;   // right
        f.planes[2] = r3 + r1;   // bottom
        f.planes[3] = r3 - r1;   // top
        f.planes[4] = r2;        // near   (0..1 clip: near = row2, not row3+row2)
        f.planes[5] = r3 - r2;   // far
        for (auto& p : f.planes) {
            const float len = glm::length(glm::vec3(p));
            if (len > 1e-8f) p /= len;
        }
        return f;
    }

    // Conservative AABB test: false only when the box is fully outside some plane (p-vertex test).
    bool intersects(const Aabb& b) const {
        if (b.empty()) return false;
        for (const glm::vec4& p : planes) {
            const glm::vec3 n{ p.x, p.y, p.z };
            // The box vertex furthest along the plane normal.
            const glm::vec3 pv{
                n.x >= 0.0f ? b.max.x : b.min.x,
                n.y >= 0.0f ? b.max.y : b.min.y,
                n.z >= 0.0f ? b.max.z : b.min.z,
            };
            if (glm::dot(n, pv) + p.w < 0.0f) return false;   // fully behind → outside
        }
        return true;
    }

    // Sphere test (center + radius): false only when fully outside some plane.
    bool intersects(const glm::vec3& center, float radius) const {
        for (const glm::vec4& p : planes)
            if (glm::dot(glm::vec3(p), center) + p.w < -radius) return false;
        return true;
    }
};

} // namespace engine::core
