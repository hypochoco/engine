//
//  renderer.h
//  engine::graphics / render
//
//  Mid-level renderer. Owns pipeline definitions, pass/target setup, and the per-frame
//  instance/uniform ring buffers. Consumes per-view render lists and records RHI commands.
//  Owns NO scene data — that flows in from the ECS as RenderViews.
//

#pragma once

#include <memory>
#include <span>

#include "engine/graphics/render/render_view.h"

namespace engine::rhi { class Device; class FrameContext; }

namespace engine::render {

class GeometryStore;

class Renderer {
public:
    Renderer(rhi::Device&, GeometryStore&);
    Renderer(Renderer&&) noexcept;
    Renderer& operator=(Renderer&&) noexcept;
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer();

    // Records the given views into the frame's command list: uploads per-view camera +
    // instance data, then for each RenderItem binds its pipeline and issues one instanced
    // indexed draw. The Device owns beginFrame/endFrame (present or readback); the Renderer
    // owns pipelinesʼ per-frame buffers + the depth target. Manages no scene data.
    void render(rhi::FrameContext& frame, std::span<const RenderView> views);

    // Enable clustered forward+ lighting. Pass a compute pipeline built from cluster.metallib
    // (the engine loads no shaders itself). When enabled, each view with point lights runs a
    // froxel light-binning compute pass before its forward pass, and the forward shader loops
    // only its cluster's lights. Pass an invalid handle to disable (loop-all fallback).
    void setClusterBinning(rhi::PipelineHandle binningPipeline);

    // Enable HDR + tone mapping. Pass a fullscreen tonemap pipeline built from tonemap.metallib
    // (color format = the final target's) and a sampler. When set, each view renders into an
    // RGBA16F HDR target, then a fullscreen tonemap pass resolves it to view.target. The scene
    // (mesh) pipeline must then target RGBA16Float. Pass an invalid pipeline to disable.
    void setTonemap(rhi::PipelineHandle tonemapPipeline, rhi::SamplerHandle sampler);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace engine::render
