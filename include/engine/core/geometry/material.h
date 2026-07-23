//
//  material.h
//  engine::core
//
//  Backend-agnostic material description (metallic-roughness workflow). CPU authoring type; the
//  render layer bakes it into render::MaterialGPU. Texture references are backend texture ids
//  (bindless slots), resolved by the graphics layer; -1 = none.
//

#pragma once

#include <glm/glm.hpp>

namespace engine {

struct Material {
    glm::vec4 baseColorFactor{1.0f};
    glm::vec3 emissiveFactor{0.0f};
    float     metallicFactor  = 0.0f;
    float     roughnessFactor = 1.0f;
    float     alphaCutoff     = 0.5f;
    bool      alphaCutout     = false;   // discard when albedo alpha < alphaCutoff

    int baseColorTexture         = -1;
    int normalTexture            = -1;
    int metallicRoughnessTexture = -1;   // glTF pack: G = roughness, B = metallic
    int emissiveTexture          = -1;
    int occlusionTexture         = -1;   // R channel
};

} // namespace engine
