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

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace engine::render
