//
//  pipeline.h
//  engine::graphics / rhi
//
//  Pipeline descriptors and the resource-interface (binding) layout. Replaces the three
//  copy-pasted Vulkan pipeline builders with one configurable descriptor. Binding is
//  bindless-first: textures live in a global table indexed from shaders; per-view uniform
//  and storage buffers are bound by slot.
//

#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "engine/graphics/rhi/types.h"
#include "engine/graphics/rhi/resources.h"

namespace engine::rhi {

// Fixed-function state (everything the old builders hardcoded), with sane defaults.
struct DepthState {
    bool      test  = true;
    bool      write = true;
    CompareOp op    = CompareOp::Less;
};

struct RasterState {
    PolygonMode polygonMode = PolygonMode::Fill;
    CullMode    cull        = CullMode::Back;
    FrontFace   frontFace   = FrontFace::CounterClockwise;
};

// -----------------------------------------------------------------------------
// Binding layout (a pipeline's resource interface)
// -----------------------------------------------------------------------------
enum class ResourceKind {
    UniformBuffer,          // small per-view constants (view/proj/lights)
    StorageBuffer,          // large arrays (instance data, materials)
    BindlessTextureTable,   // the global, unbounded sampled-texture array
    Sampler,
};

struct BindingSlot {
    uint32_t     binding = 0;
    ResourceKind kind    = ResourceKind::UniformBuffer;
    ShaderStage  stages  = ShaderStage::Vertex | ShaderStage::Fragment;
    // count == 0 marks an unbounded/bindless array (for BindlessTextureTable).
    uint32_t     count   = 1;
};

struct BindingLayout {
    std::vector<BindingSlot> slots;
    // Push-constant / small root-constant block size in bytes (0 = none).
    uint32_t pushConstantSize = 0;
};

// -----------------------------------------------------------------------------
// Pipeline descriptors
// -----------------------------------------------------------------------------
struct GraphicsPipelineDesc {
    ShaderHandle vertex;
    ShaderHandle fragment;

    VertexLayout vertexLayout;
    Topology     topology = Topology::TriangleList;
    RasterState  raster{};
    DepthState   depth{};
    BlendPreset  blend       = BlendPreset::Opaque;
    uint32_t     sampleCount = 1;

    // Render-target formats for dynamic rendering (no VkRenderPass/framebuffer objects).
    std::span<const Format> colorFormats;
    Format                  depthFormat = Format::Undefined;

    BindingLayout bindings;
    const char*   debugName = nullptr;
};

struct ComputePipelineDesc {
    ShaderHandle  compute;
    BindingLayout bindings;
    const char*   debugName = nullptr;
};

} // namespace engine::rhi
