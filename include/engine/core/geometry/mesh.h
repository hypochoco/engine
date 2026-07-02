//
//  mesh.h
//  engine::core
//
//  CPU-side mesh data and a range descriptor into shared GPU buffers.
//

#pragma once

#include <cstdint>
#include <vector>

#include "engine/core/geometry/vertex.h"

namespace engine {

// CPU-side geometry: owns its vertex and index arrays.
struct MeshData {
    std::vector<Vertex>   vertices;
    std::vector<uint32_t> indices;
};

// Describes where a mesh lives within larger shared vertex/index buffers.
// (This is what the renderer records draws against; MeshData is the source.)
struct Mesh {
    uint32_t firstVertex = 0;
    uint32_t vertexCount = 0;
    uint32_t firstIndex  = 0;
    uint32_t indexCount  = 0;
};

} // namespace engine
