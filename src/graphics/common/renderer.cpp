//
//  renderer.cpp
//  engine::graphics / render
//
//  Consumes per-view render lists: uploads the camera (view-projection) and per-instance
//  data, then issues one instanced indexed draw per RenderItem against the shared geometry.
//  Owns the per-frame camera/instance buffers and the depth target; owns no scene data.
//
//  Resource binding matches the mesh shader (Slang-assigned Metal indices): camera uniform
//  at binding 0, instance storage buffer at binding 1, vertex data at the backend's vertex
//  base index.
//

#include "engine/graphics/render/renderer.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>

#include <glm/glm.hpp>

#include "engine/graphics/rhi/device.h"
#include "engine/graphics/render/geometry_store.h"

namespace engine::render {

struct Renderer::Impl {
    rhi::Device*   device   = nullptr;
    GeometryStore* geometry = nullptr;

    rhi::BufferHandle       cameraUBO;         // glm::mat4 viewProj
    rhi::BufferHandle       instanceBuffer;    // InstanceData[]
    uint32_t                instanceCapacity = 0;
    rhi::TextureHandle      depthTex;
    rhi::RenderTargetHandle depthRT;
    uint32_t                depthW = 0, depthH = 0;

    void ensureDepth(uint32_t w, uint32_t h) {
        if (depthTex.valid() && w == depthW && h == depthH) return;
        if (depthTex.valid()) device->destroy(depthTex);
        if (depthRT.valid())  device->destroy(depthRT);
        depthTex = device->createTexture(
            { .width = w, .height = h, .format = rhi::Format::Depth32Float,
              .usage = rhi::TextureUsage::DepthTarget });
        depthRT = device->createRenderTarget(depthTex);
        depthW = w; depthH = h;
    }

    void ensureInstances(uint32_t count) {
        if (count <= instanceCapacity) return;
        const uint32_t newCap = std::max(count, instanceCapacity ? instanceCapacity * 2 : 256u);
        if (instanceBuffer.valid()) device->destroy(instanceBuffer);
        instanceBuffer = device->createBuffer(
            { .size = static_cast<uint64_t>(newCap) * sizeof(InstanceData),
              .usage = rhi::BufferUsage::Storage, .memory = rhi::MemoryMode::CpuToGpu });
        instanceCapacity = newCap;
    }
};

Renderer::Renderer(rhi::Device& device, GeometryStore& geometry) : impl_(std::make_unique<Impl>()) {
    impl_->device   = &device;
    impl_->geometry = &geometry;
    impl_->cameraUBO = device.createBuffer(
        { .size = sizeof(glm::mat4), .usage = rhi::BufferUsage::Uniform,
          .memory = rhi::MemoryMode::CpuToGpu });
}
Renderer::Renderer(Renderer&&) noexcept = default;
Renderer& Renderer::operator=(Renderer&&) noexcept = default;
Renderer::~Renderer() = default;

void Renderer::render(rhi::FrameContext& frame, std::span<const RenderView> views) {
    Impl& I = *impl_;
    rhi::CommandList cl = I.device->commandList(frame);

    for (const auto& view : views) {
        I.ensureDepth(view.width, view.height);

        const glm::mat4 viewProj = view.proj * view.view;
        I.device->updateBuffer(I.cameraUBO, 0,
                               std::as_bytes(std::span<const glm::mat4>(&viewProj, 1)));
        if (!view.instances.empty()) {
            I.ensureInstances(static_cast<uint32_t>(view.instances.size()));
            I.device->updateBuffer(I.instanceBuffer, 0, std::as_bytes(view.instances));
        }

        rhi::ColorAttachment ca;
        ca.target = view.target;
        ca.load = rhi::LoadOp::Clear; ca.store = rhi::StoreOp::Store;
        for (int i = 0; i < 4; ++i) ca.clearColor[i] = view.clearColor[i];

        rhi::DepthAttachment da;
        da.target = I.depthRT;
        da.load = rhi::LoadOp::Clear; da.store = rhi::StoreOp::DontCare;
        da.clearDepth = 1.0f;

        rhi::RenderTargetDesc rtd;
        rtd.color = std::span<const rhi::ColorAttachment>(&ca, 1);
        rtd.depth = &da;
        rtd.width = view.width; rtd.height = view.height;

        std::array<rhi::BufferBinding, 2> binds{};
        uint32_t nbinds = 0;
        binds[nbinds++] = { .binding = 0, .buffer = I.cameraUBO };
        if (I.instanceBuffer.valid()) binds[nbinds++] = { .binding = 1, .buffer = I.instanceBuffer };
        rhi::ResourceBindings rb;
        rb.buffers = std::span<const rhi::BufferBinding>(binds.data(), nbinds);

        cl.beginRendering(rtd);
        cl.setViewport(0, 0, float(view.width), float(view.height));
        cl.bindResources(rb);
        cl.bindVertexBuffer(I.geometry->vertexBuffer(), 0);
        cl.bindIndexBuffer(I.geometry->indexBuffer(), rhi::IndexType::Uint32);
        for (const auto& item : view.items) {
            cl.bindPipeline(item.pipeline);
            const auto range = I.geometry->range(item.mesh);
            cl.drawIndexed(range.indexCount, item.instanceCount,
                           range.firstIndex, range.vertexOffset, item.firstInstance);
        }
        cl.endRendering();
    }

    I.device->submit(frame, cl);
}

} // namespace engine::render
