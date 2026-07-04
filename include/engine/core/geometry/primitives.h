//
//  primitives.h
//  engine::core
//
//  Procedural mesh generators. Backend-agnostic; useful for the milestone scene
//  (sphere + plane) and for driver tests that need geometry without asset files.
//

#pragma once

#include <cstdint>

#include "engine/core/geometry/mesh.h"

namespace engine::primitives {

// Unit quad in the XY plane (z = 0), side length 1, centered at origin. Normal +Z.
MeshData makeQuad();

// Flat plane in the XZ plane (y = 0), centered at origin, `size` per side. Normal +Y.
// `subdivisions` cells per side (clamped to >= 1).
MeshData makePlane(float size = 1.0f, uint32_t subdivisions = 1);

// UV sphere centered at origin. `rings` = latitude segments (>= 2),
// `sectors` = longitude segments (>= 3). Smooth per-vertex normals.
MeshData makeSphere(float radius = 0.5f, uint32_t rings = 16, uint32_t sectors = 32);

// Axis-aligned box centered at origin with the given half-extents (per-face flat normals).
MeshData makeBox(glm::vec3 halfExtents = glm::vec3(0.5f));

} // namespace engine::primitives
