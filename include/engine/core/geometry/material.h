//
//  material.h
//  engine::core
//
//  Backend-agnostic material description. Deliberately minimal — grow it (metallic/
//  roughness, normal maps, flags) only when the renderer actually needs more.
//

#pragma once

#include <glm/glm.hpp>

namespace engine {

struct Material {
    glm::vec4 baseColorFactor{1.0f};
    // Reference to a texture, resolved by the graphics backend. -1 = none.
    int baseColorTexture = -1;
};

} // namespace engine
