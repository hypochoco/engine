//
//  device.h
//  engine::graphics / rhi
//
//  The Device owns every GPU object (buffers, textures, samplers, shaders, pipelines,
//  geometry arenas, the global bindless texture table, and the deferred-deletion queue)
//  and drives the frame lifecycle. One Device per process. Backend chosen at compile time:
//  the Impl is defined in src/graphics/metal/ or src/graphics/vulkan/, only one compiled.
//
//  Windowed vs headless differ only in whether a Swapchain is attached and what endFrame
//  does (present vs. nothing/readback).
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

#include "engine/graphics/rhi/types.h"
#include "engine/graphics/rhi/resources.h"
#include "engine/graphics/rhi/pipeline.h"
#include "engine/graphics/rhi/command_list.h"

namespace engine::rhi {

// Opaque handle to a platform window (e.g. wraps a GLFW window / native NSWindow).
// Only needed for windowed presentation; headless never uses it.
struct WindowSurface {
    void* nativeWindow = nullptr;   // e.g. GLFWwindow*
    uint32_t width  = 0;
    uint32_t height = 0;
};

struct DeviceConfig {
    bool        enableValidation = false;
    uint32_t    framesInFlight   = 2;   // fixes the old MAX_FRAMES_IN_FLIGHT = 1 stall
    const char* appName          = "engine";
};

// Per-frame token: waits the frame's fence on acquire, carries the command list and (in
// windowed mode) the acquired swapchain image. Move-only.
class FrameContext {
public:
    struct Impl;   // backend-defined (pimpl); public so the backend can cross-reference it

    FrameContext() = default;
    FrameContext(FrameContext&&) noexcept;
    FrameContext& operator=(FrameContext&&) noexcept;
    FrameContext(const FrameContext&) = delete;
    FrameContext& operator=(const FrameContext&) = delete;
    ~FrameContext();

    uint32_t frameIndex() const;               // 0 .. framesInFlight-1
    RenderTargetHandle swapchainTarget() const; // invalid in headless

private:
    friend class Device;
    std::unique_ptr<Impl> impl_;
};

class Swapchain {
public:
    uint32_t width() const;
    uint32_t height() const;
    Format   format() const;
    void     resize(uint32_t width, uint32_t height);
private:
    friend class Device;
    struct Impl;
    Impl* impl_ = nullptr;   // owned by the Device
};

class Device {
public:
    struct Impl;   // backend-defined (pimpl); public so the backend can cross-reference it

    static Device createHeadless(const DeviceConfig& = {});
    static Device createWindowed(const WindowSurface&, const DeviceConfig& = {});

    Device(Device&&) noexcept;
    Device& operator=(Device&&) noexcept;
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;
    ~Device();

    // --- resource creation (Device owns the objects; returns handles) ---
    BufferHandle   createBuffer(const BufferDesc&, std::span<const std::byte> initialData = {});
    TextureHandle  createTexture(const TextureDesc&, std::span<const std::byte> initialData = {});
    SamplerHandle  createSampler(const SamplerDesc&);
    ShaderHandle   createShader(std::span<const std::byte> blob, ShaderStage);
    PipelineHandle createGraphicsPipeline(const GraphicsPipelineDesc&);
    PipelineHandle createComputePipeline(const ComputePipelineDesc&);
    // Offscreen/headless render target from an existing texture.
    RenderTargetHandle createRenderTarget(TextureHandle);

    // --- updates ---
    void  updateBuffer(BufferHandle, uint64_t offset, std::span<const std::byte>);
    void* map(BufferHandle);            // CpuToGpu buffers: persistent-mapped ring pointer
    void  unmap(BufferHandle);

    // --- bindless textures ---
    // Registers a texture in the global bindless table and returns its shader-visible index.
    uint32_t registerBindlessTexture(TextureHandle);
    void     unregisterBindlessTexture(uint32_t index);

    // --- deferred, fence-gated destruction ---
    void destroy(BufferHandle);
    void destroy(TextureHandle);
    void destroy(SamplerHandle);
    void destroy(ShaderHandle);
    void destroy(PipelineHandle);
    void destroy(RenderTargetHandle);

    // --- frame lifecycle ---
    FrameContext beginFrame();
    CommandList  commandList(FrameContext&);
    void         submit(FrameContext&, CommandList&);
    void         endFrame(FrameContext&&);   // present (windowed) or finish/readback (headless)
    void         waitIdle();

    // Copy a render target / texture back to CPU (headless offline rendering).
    void readback(TextureHandle, std::span<std::byte> out);

    Swapchain* swapchain();   // nullptr in headless

private:
    Device();
    std::unique_ptr<Impl> impl_;
};

} // namespace engine::rhi
