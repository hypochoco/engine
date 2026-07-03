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

namespace engine::rhi { class Device; }

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

    // Uploads per-frame instance/uniform data, then for each view: begin pass, bind
    // pipeline (on change), bind the shared geometry + per-view resources, and issue one
    // instanced indexed draw per RenderItem. A straight swap to drawIndexedIndirect later
    // leaves this signature unchanged.
    void render(std::span<const RenderView> views);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace engine::render
