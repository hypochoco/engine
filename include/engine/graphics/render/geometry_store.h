//
//  geometry_store.h
//  engine::graphics / render
//
//  Owns the shared vertex/index arenas (a couple of big rhi::Buffers) and sub-allocates
//  Mesh ranges from them. This is the scalable geometry ownership model from
//  notes/investigations/2026-07-02-geometry-scaling.md: core::MeshData is transient loader
//  output; the authoritative runtime store is these pooled buffers + MeshHandles.
//

#pragma once

#include <cstdint>
#include <memory>

#include "engine/core/geometry/mesh.h"
#include "engine/graphics/rhi/types.h"
#include "engine/graphics/rhi/resources.h"
#include "engine/graphics/view/render_view.h"

namespace engine::rhi { class Device; }

namespace engine::render {

// rhi::VertexLayout describing engine::Vertex (position/normal/uv/color). Defined in the
// render layer because rhi must not depend on core.
rhi::VertexLayout coreVertexLayout();

class GeometryStore {
public:
    explicit GeometryStore(rhi::Device&);
    GeometryStore(GeometryStore&&) noexcept;
    GeometryStore& operator=(GeometryStore&&) noexcept;
    GeometryStore(const GeometryStore&) = delete;
    GeometryStore& operator=(const GeometryStore&) = delete;
    ~GeometryStore();

    // Appends CPU geometry to the arenas (growing/suballocating) and returns a handle.
    MeshHandle upload(const MeshData&);

    // Draw parameters for a mesh (into the shared index/vertex buffers).
    struct DrawRange {
        uint32_t indexCount    = 0;
        uint32_t firstIndex    = 0;
        int32_t  vertexOffset  = 0;
    };
    DrawRange range(MeshHandle) const;

    rhi::BufferHandle vertexBuffer() const;
    rhi::BufferHandle indexBuffer() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace engine::render
