//
//  bounds.h
//  engine::core / geometry
//
//  Local-space bounding-box computation for a MeshData. Kept in geometry (which knows Vertex) and
//  separate from math/bounds.h (which is glm-only) to keep the math layer dependency-free.
//

#pragma once

#include "engine/core/geometry/mesh.h"
#include "engine/core/math/bounds.h"

namespace engine {

// Tight local-space AABB over a mesh's vertex positions (empty box for an empty mesh).
inline core::Aabb computeBounds(const MeshData& mesh) {
    core::Aabb b;
    for (const Vertex& v : mesh.vertices) b.expand(v.position);
    return b;
}

} // namespace engine
