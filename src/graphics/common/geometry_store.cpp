//
//  geometry_store.cpp
//  engine::graphics / render
//
//  Owns shared vertex/index arenas backed by rhi::Buffers and hands out MeshHandles that
//  index draw ranges. First-cut implementation: accumulate CPU-side and rebuild the GPU
//  buffers on each upload. Uploads happen at load time, not per frame, so this is fine for
//  now; a suballocating/growing arena is the scaling-note follow-up (indices stay mesh-local
//  and are offset at draw time via baseVertex, so no re-biasing is needed).
//

#include "engine/graphics/render/geometry_store.h"

#include <cstddef>
#include <span>

#include "engine/graphics/rhi/device.h"

namespace engine::render {

rhi::VertexLayout coreVertexLayout() {
    rhi::VertexLayout v;
    v.stride = static_cast<uint32_t>(sizeof(Vertex));
    v.attributes = {
        { 0, rhi::VertexFormat::Float3, static_cast<uint32_t>(offsetof(Vertex, position)) },
        { 1, rhi::VertexFormat::Float3, static_cast<uint32_t>(offsetof(Vertex, normal)) },
        { 2, rhi::VertexFormat::Float2, static_cast<uint32_t>(offsetof(Vertex, uv)) },
        { 3, rhi::VertexFormat::Float3, static_cast<uint32_t>(offsetof(Vertex, color)) },
        { 4, rhi::VertexFormat::Float4, static_cast<uint32_t>(offsetof(Vertex, tangent)) },
    };
    return v;
}

struct GeometryStore::Impl {
    rhi::Device*          device = nullptr;
    std::vector<Vertex>   vertices;
    std::vector<uint32_t> indices;
    std::vector<DrawRange> ranges;
    rhi::BufferHandle     vbuf;
    rhi::BufferHandle     ibuf;

    void rebuild() {
        if (vbuf.valid()) device->destroy(vbuf);
        if (ibuf.valid()) device->destroy(ibuf);
        vbuf = device->createBuffer(
            { .size = vertices.size() * sizeof(Vertex),
              .usage = rhi::BufferUsage::Vertex, .memory = rhi::MemoryMode::CpuToGpu },
            std::as_bytes(std::span<const Vertex>(vertices)));
        ibuf = device->createBuffer(
            { .size = indices.size() * sizeof(uint32_t),
              .usage = rhi::BufferUsage::Index, .memory = rhi::MemoryMode::CpuToGpu },
            std::as_bytes(std::span<const uint32_t>(indices)));
    }
};

GeometryStore::GeometryStore(rhi::Device& device) : impl_(std::make_unique<Impl>()) {
    impl_->device = &device;
}
GeometryStore::GeometryStore(GeometryStore&&) noexcept = default;
GeometryStore& GeometryStore::operator=(GeometryStore&&) noexcept = default;
GeometryStore::~GeometryStore() = default;

MeshHandle GeometryStore::upload(const MeshData& mesh) {
    Impl& I = *impl_;
    DrawRange r;
    r.vertexOffset = static_cast<int32_t>(I.vertices.size());
    r.firstIndex   = static_cast<uint32_t>(I.indices.size());
    r.indexCount   = static_cast<uint32_t>(mesh.indices.size());
    I.vertices.insert(I.vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
    I.indices.insert(I.indices.end(),  mesh.indices.begin(),  mesh.indices.end());
    I.ranges.push_back(r);
    I.rebuild();
    return MeshHandle{ static_cast<uint32_t>(I.ranges.size() - 1), 0 };
}

GeometryStore::DrawRange GeometryStore::range(MeshHandle h) const { return impl_->ranges[h.index]; }
rhi::BufferHandle GeometryStore::vertexBuffer() const { return impl_->vbuf; }
rhi::BufferHandle GeometryStore::indexBuffer() const  { return impl_->ibuf; }

} // namespace engine::render
