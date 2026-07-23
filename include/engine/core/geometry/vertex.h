//
//  vertex.h
//  engine::core
//
//  Backend-agnostic vertex data. No Vulkan/Metal binding descriptions live here — each
//  graphics backend derives its own vertex layout from this pure-data struct.
//

#pragma once

#include <glm/glm.hpp>

namespace engine {

// Canonical vertex layout used across the engine.
// Fields are ordered position/normal/uv/color/tangent; extend deliberately (adding fields
// touches the vertex layout + shaders). Skinning weights are intentionally omitted until a
// feature needs them.
struct Vertex {
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f};
    glm::vec2 uv{0.0f};
    glm::vec3 color{1.0f};
    // Tangent frame for normal mapping: xyz = surface tangent (aligned with +U), w = ±1
    // handedness so the bitangent is w * cross(normal, tangent). w = 0 means "no tangent"
    // (untextured / normal mapping off), which is the default for procedurally-generated meshes.
    glm::vec4 tangent{0.0f};

    bool operator==(const Vertex& other) const {
        return position == other.position
            && normal   == other.normal
            && uv       == other.uv
            && color    == other.color
            && tangent  == other.tangent;
    }
};

} // namespace engine
