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
// Fields are ordered position/normal/uv/color; extend deliberately (adding fields
// touches shaders). Tangents / skinning weights are intentionally omitted until a
// feature needs them.
struct Vertex {
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f};
    glm::vec2 uv{0.0f};
    glm::vec3 color{1.0f};

    bool operator==(const Vertex& other) const {
        return position == other.position
            && normal   == other.normal
            && uv       == other.uv
            && color    == other.color;
    }
};

} // namespace engine
