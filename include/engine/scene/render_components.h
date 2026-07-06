//
//  render_components.h
//  engine::scene
//
//  ECS components that reference render resources. These live in the scene bridge (not in
//  ecs or the Renderer) so that `ecs` stays graphics-free and the `Renderer` stays ECS-free —
//  the scene layer is the only thing that knows about both. Trivially copyable (ECS rule).
//

#pragma once

#include <cstdint>

#include "engine/graphics/view/render_view.h"   // render::MeshHandle

namespace engine::scene {

struct RenderMesh {
    render::MeshHandle mesh;
};

struct RenderMaterial {
    uint32_t materialIndex = 0;   // index into the app's materials array
};

} // namespace engine::scene
