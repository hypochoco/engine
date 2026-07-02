//
//  model.h
//  engine::core
//
//  A loaded model: its meshes, materials, and the mapping between them.
//

#pragma once

#include <cstdint>
#include <vector>

#include "engine/core/geometry/mesh.h"
#include "engine/core/geometry/material.h"

namespace engine {

struct ModelData {
    std::vector<MeshData> meshes;
    std::vector<Material> materials;
    // Parallel to `meshes`: submesh i uses materials[meshMaterial[i]].
    std::vector<uint32_t> meshMaterial;
};

} // namespace engine
