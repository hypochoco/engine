//
//  command_list.h
//  engine::graphics / rhi
//
//  Coarse command recording. Maps to a VkCommandBuffer (dynamic rendering) or a set of
//  MTL encoders. The API is intentionally batch-granular: no per-instance virtual calls.
//

#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "engine/graphics/rhi/types.h"

namespace engine::rhi {

// One color attachment for a rendering pass.
struct ColorAttachment {
    RenderTargetHandle target;          // offscreen texture view or swapchain image
    RenderTargetHandle resolveTarget;   // MSAA resolve destination (single-sample); invalid = none
    LoadOp  load  = LoadOp::Clear;
    StoreOp store = StoreOp::Store;
    float   clearColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
};

struct DepthAttachment {
    RenderTargetHandle target;
    LoadOp  load  = LoadOp::Clear;
    StoreOp store = StoreOp::DontCare;
    float   clearDepth = 1.0f;
};

// Describes a rendering pass (MRT-capable for deferred). Corresponds to a
// begin/endRendering scope (VK dynamic rendering / MTLRenderPassDescriptor).
struct RenderTargetDesc {
    std::span<const ColorAttachment> color;   // 1+ for MRT / G-buffer
    const DepthAttachment*           depth = nullptr;
    uint32_t width  = 0;
    uint32_t height = 0;
};

// -----------------------------------------------------------------------------
// Resource bindings for a pass (bound once per view, not per draw)
// -----------------------------------------------------------------------------
struct BufferBinding {
    uint32_t     binding = 0;
    BufferHandle buffer;
    uint64_t     offset = 0;
    uint64_t     range  = ~0ull;   // whole buffer
};

// A sampled texture bound at an explicit slot (non-bindless; for post/compute passes that sample
// a specific target, e.g. tone-mapping the HDR buffer). Bindless material textures are separate.
struct TextureBinding {
    uint32_t      binding = 0;
    TextureHandle texture;
};

struct SamplerBinding {
    uint32_t      binding = 0;
    SamplerHandle sampler;
};

// The bindless texture table is owned + bound globally by the Device; per-view bindings
// carry the uniform/storage buffers, plus explicitly-bound sampled textures + sampler(s).
struct ResourceBindings {
    std::span<const BufferBinding>  buffers;
    std::span<const TextureBinding> textures;
    std::span<const SamplerBinding> samplers;
};

// -----------------------------------------------------------------------------
// Resource state transitions (explicit barriers for multi-pass graphs)
// -----------------------------------------------------------------------------
// A pass that samples a texture a prior pass wrote (or reads a buffer a compute pass wrote)
// needs a dependency between them. Vulkan REQUIRES an explicit barrier + image-layout
// transition; Metal auto-tracks hazards for tracked resources within a command buffer, so the
// same call is a near-no-op there. The render graph derives these from declared pass reads/
// writes and emits them at pass boundaries — keeping the contract explicit (correct on Vulkan)
// without penalizing Metal.
enum class ResourceState {
    Undefined,
    RenderTarget,   // color/depth attachment being written
    ShaderRead,     // sampled / read in a shader
    StorageRW,      // read-write storage (compute)
    TransferSrc,
    TransferDst,
    Present,        // swapchain image ready to present
};

struct ResourceTransition {
    TextureHandle texture;
    ResourceState from = ResourceState::Undefined;
    ResourceState to   = ResourceState::Undefined;
};

class CommandList {
public:
    struct Impl;   // backend-defined (pimpl); public so the backend can cross-reference it

    void beginRendering(const RenderTargetDesc&);
    void endRendering();

    // Compute scope (outside a rendering scope). Between beginCompute()/endCompute(), bindPipeline
    // binds a compute pipeline, bindResources binds storage/uniform buffers to the compute stage,
    // and dispatch() launches threadgroups. The render graph wraps compute passes in these.
    void beginCompute();
    void endCompute();

    // Insert resource-state transitions (see ResourceTransition). Call OUTSIDE a rendering
    // scope (at a pass boundary). No-op / lightweight on auto-tracked Metal; real barriers on
    // Vulkan. Safe to call with an empty span.
    void resourceBarrier(std::span<const ResourceTransition>);

    void bindPipeline(PipelineHandle);
    void bindVertexBuffer(BufferHandle, uint32_t slot = 0);
    void bindIndexBuffer(BufferHandle, IndexType);
    void bindResources(const ResourceBindings&);
    void setPushConstants(std::span<const std::byte>);

    void setViewport(float x, float y, float width, float height,
                     float minDepth = 0.0f, float maxDepth = 1.0f);
    void setScissor(int32_t x, int32_t y, uint32_t width, uint32_t height);

    void draw(uint32_t vertexCount, uint32_t instanceCount,
              uint32_t firstVertex = 0, uint32_t firstInstance = 0);
    void drawIndexed(uint32_t indexCount, uint32_t instanceCount,
                     uint32_t firstIndex = 0, int32_t vertexOffset = 0,
                     uint32_t firstInstance = 0);
    // GPU-driven path: args come from a buffer (BufferUsage::Indirect).
    void drawIndexedIndirect(BufferHandle args, uint64_t offset, uint32_t drawCount,
                             uint32_t stride);

    void dispatch(uint32_t groupsX, uint32_t groupsY, uint32_t groupsZ);

private:
    friend class Device;
    Impl* impl_ = nullptr;   // borrowed; owned by the Device / FrameContext
};

} // namespace engine::rhi
