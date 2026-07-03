//
//  render_view.h
//  engine::graphics / render
//
//  The ECS-facing contract. An extraction system turns world state into these plain-data
//  render lists each frame; the Renderer consumes them. No Vk*/MTL:: and no ECS types here.
//

#pragma once

#include <cstdint>
#include <span>

#include <glm/glm.hpp>

#include "engine/graphics/rhi/types.h"

namespace engine::render {

// Handle into the GeometryStore's shared vertex/index arenas.
struct MeshTag;
using MeshHandle = rhi::Handle<MeshTag>;

// Index into the per-frame materials storage buffer.
using MaterialHandle = uint32_t;

// One batchable unit: a contiguous run of instances sharing a pipeline + mesh + material.
// Becomes one (indexed, instanced) draw.
struct RenderItem {
    MeshHandle         mesh;
    rhi::PipelineHandle pipeline;
    uint32_t           firstInstance = 0;
    uint32_t           instanceCount = 0;
};

// Per-instance record, SoA-friendly, uploaded into the per-frame instance storage buffer.
// std140/std430-friendly layout (16-byte aligned; matrices then scalars + padding).
struct InstanceData {
    glm::mat4 model{1.0f};
    glm::mat4 normalModel{1.0f};   // for correct normals under non-uniform scale
    uint32_t  materialIndex = 0;
    uint32_t  _pad[3]       = {0, 0, 0};
};

// A single view/pass to render. Multiple views enable multi-pass (deferred G-buffer →
// lighting), multiple cameras, or many parallel-env targets.
struct RenderView {
    glm::mat4 view{1.0f};
    glm::mat4 proj{1.0f};
    rhi::RenderTargetHandle target;          // swapchain image OR offscreen texture(s)
    uint32_t  width  = 0;                     // target dimensions (viewport + depth sizing)
    uint32_t  height = 0;
    float     clearColor[4] = {0.08f, 0.10f, 0.14f, 1.0f};

    std::span<const RenderItem>   items;      // pre-sorted by pipeline → mesh → material
    std::span<const InstanceData> instances;  // indexed by RenderItem instance ranges
};

} // namespace engine::render
