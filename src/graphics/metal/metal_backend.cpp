//
//  metal_backend.cpp
//  engine::graphics / metal
//
//  Metal implementation of the RHI. Also the single translation unit that defines the
//  metal-cpp implementation macros (exactly one TU may, per metal-cpp README). Pure C++.
//
//  Covers: headless Device (MTL::Device + queue), handle-addressed pools (buffers, textures,
//  render targets, shaders, pipelines), shader libraries loaded from a Slang-compiled
//  metallib, graphics pipeline state (with a vertex descriptor), the offscreen frame +
//  render-encoder lifecycle, and CPU readback. Swapchain/present + bindless come next.
//

// --- metal-cpp implementation macros (exactly one TU may define these) ---
#define NS_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include <dispatch/dispatch.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "engine/graphics/rhi/rhi.h"

// Objective-C++ window shim (src/graphics/metal/metal_window.mm): attaches a CAMetalLayer to
// a GLFW window and returns it as an opaque pointer (a metal-cpp CA::MetalLayer*).
extern "C" void* engine_metal_create_layer(void* glfwWindow, void* mtlDevice,
                                           uint32_t width, uint32_t height);

namespace engine::rhi {

const char* backendName() { return "Metal"; }

// -----------------------------------------------------------------------------
// enum mapping helpers
// -----------------------------------------------------------------------------
namespace {

MTL::ResourceOptions storageOptions(MemoryMode mode) {
    switch (mode) {
        case MemoryMode::GpuOnly:  return MTL::ResourceStorageModePrivate;
        case MemoryMode::CpuToGpu: return MTL::ResourceStorageModeShared;
        case MemoryMode::GpuToCpu: return MTL::ResourceStorageModeShared;
    }
    return MTL::ResourceStorageModeShared;
}

MTL::PixelFormat toMTLPixelFormat(Format f) {
    switch (f) {
        case Format::R8Unorm:         return MTL::PixelFormatR8Unorm;
        case Format::RG8Unorm:        return MTL::PixelFormatRG8Unorm;
        case Format::RGBA8Unorm:      return MTL::PixelFormatRGBA8Unorm;
        case Format::RGBA8Srgb:       return MTL::PixelFormatRGBA8Unorm_sRGB;
        case Format::BGRA8Unorm:      return MTL::PixelFormatBGRA8Unorm;
        case Format::RGBA16Float:     return MTL::PixelFormatRGBA16Float;
        case Format::RGBA32Float:     return MTL::PixelFormatRGBA32Float;
        case Format::Depth32Float:    return MTL::PixelFormatDepth32Float;
        case Format::Depth24Stencil8: return MTL::PixelFormatDepth32Float_Stencil8;
        case Format::Undefined:       return MTL::PixelFormatInvalid;
    }
    return MTL::PixelFormatInvalid;
}

MTL::VertexFormat toMTLVertexFormat(VertexFormat f) {
    switch (f) {
        case VertexFormat::Float:      return MTL::VertexFormatFloat;
        case VertexFormat::Float2:     return MTL::VertexFormatFloat2;
        case VertexFormat::Float3:     return MTL::VertexFormatFloat3;
        case VertexFormat::Float4:     return MTL::VertexFormatFloat4;
        case VertexFormat::UByte4Norm: return MTL::VertexFormatUChar4Normalized;
    }
    return MTL::VertexFormatFloat3;
}

MTL::PrimitiveType toMTLPrimitive(Topology t) {
    switch (t) {
        case Topology::TriangleList:
        case Topology::TriangleStrip: return MTL::PrimitiveTypeTriangle;
        case Topology::LineList:      return MTL::PrimitiveTypeLine;
        case Topology::PointList:     return MTL::PrimitiveTypePoint;
    }
    return MTL::PrimitiveTypeTriangle;
}

MTL::LoadAction toMTLLoad(LoadOp op) {
    switch (op) {
        case LoadOp::Load:     return MTL::LoadActionLoad;
        case LoadOp::Clear:    return MTL::LoadActionClear;
        case LoadOp::DontCare: return MTL::LoadActionDontCare;
    }
    return MTL::LoadActionClear;
}

MTL::StoreAction toMTLStore(StoreOp op) {
    switch (op) {
        case StoreOp::Store:    return MTL::StoreActionStore;
        case StoreOp::DontCare: return MTL::StoreActionDontCare;
    }
    return MTL::StoreActionStore;
}

MTL::CompareFunction toMTLCompare(CompareOp op) {
    switch (op) {
        case CompareOp::Never:        return MTL::CompareFunctionNever;
        case CompareOp::Less:         return MTL::CompareFunctionLess;
        case CompareOp::Equal:        return MTL::CompareFunctionEqual;
        case CompareOp::LessEqual:    return MTL::CompareFunctionLessEqual;
        case CompareOp::Greater:      return MTL::CompareFunctionGreater;
        case CompareOp::NotEqual:     return MTL::CompareFunctionNotEqual;
        case CompareOp::GreaterEqual: return MTL::CompareFunctionGreaterEqual;
        case CompareOp::Always:       return MTL::CompareFunctionAlways;
    }
    return MTL::CompareFunctionLess;
}

MTL::IndexType toMTLIndexType(IndexType t) {
    return t == IndexType::Uint16 ? MTL::IndexTypeUInt16 : MTL::IndexTypeUInt32;
}

bool isDepthFormat(Format f) {
    return f == Format::Depth32Float || f == Format::Depth24Stencil8;
}

bool has(TextureUsage v, TextureUsage bit) { return static_cast<uint32_t>(v & bit) != 0; }

// Metal buffer-index convention: low indices (0..N) hold resources (uniform/storage buffers,
// matching what Slang assigns, e.g. the camera uniform at buffer(0)); vertex buffers are bound
// at a high base so they never collide with resources. RHI vertex slot s -> Metal index base+s.
constexpr uint32_t kVertexBufferBase = 16;

} // namespace

// -----------------------------------------------------------------------------
// Device::Impl — the backend state and all resource pools
// -----------------------------------------------------------------------------
struct Device::Impl {
    NS::SharedPtr<MTL::Device>       device;
    NS::SharedPtr<MTL::CommandQueue> queue;
    DeviceConfig                     config;

    // Frames-in-flight throttle: a counting semaphore (value = framesInFlight) waited in
    // beginFrame and signaled from each command buffer's completion handler, so the CPU never
    // gets more than framesInFlight frames ahead of the GPU. frameCounter cycles the frame
    // index (0..framesInFlight-1) that the render layer's ring allocator keys on.
    dispatch_semaphore_t frameSem = nullptr;
    uint64_t             frameCounter = 0;

    // Block until every in-flight frame's completion handler has signaled the throttle semaphore,
    // restoring it to its initial count. Windowed frames signal asynchronously, so this is needed
    // both as waitIdle() and before releasing the semaphore — libdispatch TRAPS if a semaphore is
    // released while its value is below the value it was created with ("still in use").
    void drainFramesInFlight() {
        if (!frameSem) return;
        const uint32_t fif = std::max(config.framesInFlight, 1u);
        for (uint32_t i = 0; i < fif; ++i) dispatch_semaphore_wait(frameSem, DISPATCH_TIME_FOREVER);
        for (uint32_t i = 0; i < fif; ++i) dispatch_semaphore_signal(frameSem);
    }

    ~Impl() { if (frameSem) { drainFramesInFlight(); dispatch_release(frameSem); } }

    // Windowed swapchain (CAMetalLayer); unused in headless mode.
    CA::MetalLayer* layer      = nullptr;   // retained by the .mm shim
    bool            windowed   = false;
    uint32_t        swapWidth  = 0;
    uint32_t        swapHeight = 0;
    uint32_t        swapTexSlot = 0xFFFF'FFFFu;   // reserved texture slot for the drawable
    uint32_t        swapRTSlot  = 0xFFFF'FFFFu;   // reserved render-target slot

    struct BufferSlot  { NS::SharedPtr<MTL::Buffer>  buffer;  uint32_t generation = 0; bool alive = false; };
    struct TextureSlot { NS::SharedPtr<MTL::Texture> texture; uint32_t generation = 0; bool alive = false; };
    struct ShaderSlot  { NS::SharedPtr<MTL::Library> library; NS::SharedPtr<MTL::Function> function;
                         uint32_t generation = 0; bool alive = false; };
    struct PipelineSlot{ NS::SharedPtr<MTL::RenderPipelineState> state;
                         NS::SharedPtr<MTL::DepthStencilState> depthState;
                         NS::SharedPtr<MTL::ComputePipelineState> computeState;
                         bool isCompute = false;
                         MTL::PrimitiveType topology = MTL::PrimitiveTypeTriangle;
                         MTL::CullMode cull = MTL::CullModeNone;
                         MTL::Winding  winding = MTL::WindingClockwise;   // framebuffer-space front winding
                         uint32_t generation = 0; bool alive = false; };
    struct RenderTargetSlot { uint32_t textureIndex = 0; uint32_t generation = 0; bool alive = false; };
    struct SamplerSlot { NS::SharedPtr<MTL::SamplerState> sampler; uint32_t generation = 0; bool alive = false; };

    std::vector<BufferSlot>       buffers;
    std::vector<TextureSlot>      textures;
    std::vector<ShaderSlot>       shaders;
    std::vector<PipelineSlot>     pipelines;
    std::vector<RenderTargetSlot> renderTargets;
    std::vector<SamplerSlot>      samplers;

    template <class Vec>
    static uint32_t acquire(Vec& pool, std::vector<uint32_t>& freeList) {
        if (!freeList.empty()) { uint32_t i = freeList.back(); freeList.pop_back(); return i; }
        pool.emplace_back();
        return static_cast<uint32_t>(pool.size() - 1);
    }
    std::vector<uint32_t> freeBuffers, freeShaders, freePipelines, freeRenderTargets, freeSamplers;
    std::vector<uint32_t> freeTextures;

    // Global bindless texture table: slot → texture pool index (0xFFFFFFFF = empty). Materials
    // store a slot; CommandList::bindBindlessTextures binds each alive slot's texture to the
    // fragment stage. Kept sparse-friendly with a free list; sized on demand up to kMax.
    std::vector<uint32_t> bindlessSlots;      // slot -> texture pool index (or ~0u)
    std::vector<uint32_t> freeBindlessSlots;
};

// -----------------------------------------------------------------------------
// CommandList::Impl — borrowed per-frame recording state
// -----------------------------------------------------------------------------
struct CommandList::Impl {
    Device::Impl*                dev = nullptr;
    MTL::CommandBuffer*          cmd = nullptr;      // borrowed (autoreleased, owned by frame pool)
    MTL::RenderCommandEncoder*   encoder = nullptr;  // borrowed (autoreleased)
    MTL::ComputeCommandEncoder*  computeEncoder = nullptr;  // borrowed (autoreleased); compute scope
    MTL::PrimitiveType           topology = MTL::PrimitiveTypeTriangle;
    MTL::Buffer*                 indexBuffer = nullptr;   // borrowed
    MTL::IndexType               indexType = MTL::IndexTypeUInt32;
};

// -----------------------------------------------------------------------------
// FrameContext
// -----------------------------------------------------------------------------
struct FrameContext::Impl {
    NS::AutoreleasePool* pool = nullptr;
    MTL::CommandBuffer*  cmd  = nullptr;   // borrowed (autoreleased in pool)
    CommandList::Impl    cmdState;
    uint32_t             frameIndex = 0;
    CA::MetalDrawable*   drawable = nullptr;    // windowed: current swapchain drawable
    RenderTargetHandle   swapchainRT{};         // windowed: target for `drawable`
};

FrameContext::FrameContext(FrameContext&&) noexcept = default;
FrameContext& FrameContext::operator=(FrameContext&&) noexcept = default;
FrameContext::~FrameContext() {
    if (impl_ && impl_->pool) { impl_->pool->release(); impl_->pool = nullptr; }
}
uint32_t FrameContext::frameIndex() const { return impl_ ? impl_->frameIndex : 0; }
RenderTargetHandle FrameContext::swapchainTarget() const { return impl_ ? impl_->swapchainRT : RenderTargetHandle{}; }

// -----------------------------------------------------------------------------
// Device lifetime
// -----------------------------------------------------------------------------
Device::Device() = default;
Device::Device(Device&&) noexcept = default;
Device& Device::operator=(Device&&) noexcept = default;
Device::~Device() = default;

Device Device::createHeadless(const DeviceConfig& config) {
    Device d;
    d.impl_ = std::make_unique<Impl>();
    d.impl_->config = config;
    d.impl_->device = NS::TransferPtr(MTL::CreateSystemDefaultDevice());
    if (d.impl_->device) {
        d.impl_->queue = NS::TransferPtr(d.impl_->device->newCommandQueue());
    }
    d.impl_->frameSem = dispatch_semaphore_create(std::max(config.framesInFlight, 1u));
    return d;
}

Device Device::createWindowed(const WindowSurface& surface, const DeviceConfig& config) {
    Device d;
    d.impl_ = std::make_unique<Impl>();
    d.impl_->config = config;
    d.impl_->device = NS::TransferPtr(MTL::CreateSystemDefaultDevice());
    d.impl_->queue  = NS::TransferPtr(d.impl_->device->newCommandQueue());
    d.impl_->frameSem = dispatch_semaphore_create(std::max(config.framesInFlight, 1u));

    // Attach a CAMetalLayer to the window (Objective-C++ shim) and reserve swapchain slots.
    void* layerPtr = engine_metal_create_layer(surface.nativeWindow, d.impl_->device.get(),
                                               surface.width, surface.height);
    d.impl_->layer      = reinterpret_cast<CA::MetalLayer*>(layerPtr);
    d.impl_->windowed   = true;
    d.impl_->swapWidth  = surface.width;
    d.impl_->swapHeight = surface.height;

    Impl& I = *d.impl_;
    I.swapTexSlot = Impl::acquire(I.textures, I.freeTextures);
    I.textures[I.swapTexSlot].alive = true;
    I.swapRTSlot = Impl::acquire(I.renderTargets, I.freeRenderTargets);
    I.renderTargets[I.swapRTSlot].textureIndex = I.swapTexSlot;
    I.renderTargets[I.swapRTSlot].alive = true;
    return d;
}

// -----------------------------------------------------------------------------
// Buffers
// -----------------------------------------------------------------------------
BufferHandle Device::createBuffer(const BufferDesc& desc, std::span<const std::byte> initialData) {
    Impl& I = *impl_;
    MTL::ResourceOptions opts = storageOptions(desc.memory);
    MTL::Buffer* raw = nullptr;
    if (!initialData.empty()) {
        opts = MTL::ResourceStorageModeShared;   // TODO(metal): staging blit for Private + data
        raw  = I.device->newBuffer(initialData.data(), static_cast<NS::UInteger>(desc.size), opts);
    } else {
        raw = I.device->newBuffer(static_cast<NS::UInteger>(desc.size), opts);
    }
    uint32_t idx = Impl::acquire(I.buffers, I.freeBuffers);
    I.buffers[idx].buffer = NS::TransferPtr(raw);
    I.buffers[idx].alive  = true;
    return BufferHandle{ idx, I.buffers[idx].generation };
}

void Device::destroy(BufferHandle h) {
    if (!h.valid() || !impl_ || h.index >= impl_->buffers.size()) return;
    auto& s = impl_->buffers[h.index];
    if (!s.alive || s.generation != h.generation) return;
    s.buffer.reset();
    s.alive = false;
    ++s.generation;
    impl_->freeBuffers.push_back(h.index);
}

void* Device::map(BufferHandle h) {
    if (!h.valid() || h.index >= impl_->buffers.size()) return nullptr;
    auto& s = impl_->buffers[h.index];
    if (!s.alive || s.generation != h.generation) return nullptr;
    return s.buffer->contents();
}
void Device::unmap(BufferHandle) { /* Shared storage: nothing to do */ }

void Device::updateBuffer(BufferHandle h, uint64_t offset, std::span<const std::byte> data) {
    if (void* p = map(h)) {
        std::memcpy(static_cast<uint8_t*>(p) + offset, data.data(), data.size());
    }
}

// -----------------------------------------------------------------------------
// Textures & render targets
// -----------------------------------------------------------------------------
TextureHandle Device::createTexture(const TextureDesc& desc, std::span<const std::byte> initialData) {
    Impl& I = *impl_;
    auto* td = MTL::TextureDescriptor::alloc()->init();
    const bool msaa = desc.sampleCount > 1;
    td->setTextureType(msaa ? MTL::TextureType2DMultisample : MTL::TextureType2D);
    if (msaa) td->setSampleCount(desc.sampleCount);
    td->setPixelFormat(toMTLPixelFormat(desc.format));
    td->setWidth(desc.width);
    td->setHeight(desc.height);
    td->setMipmapLevelCount(desc.mipLevels);
    // Depth targets must be Private on Apple Silicon; color targets stay Shared so we can
    // read them back on the headless path. TODO(metal): Private + blit for GPU-only color.
    // A transient render-target-only texture (not sampled, not read back) can live purely in
    // on-chip tile memory (Memoryless) — a TBDR bandwidth win, and the RHI's `transient` hint.
    // MSAA targets are never Shared/readable (you read the resolve): Memoryless if transient,
    // else Private.
    const bool renderTargetOnly =
        (has(desc.usage, TextureUsage::ColorTarget) || has(desc.usage, TextureUsage::DepthTarget)) &&
        !has(desc.usage, TextureUsage::Sampled) &&
        !has(desc.usage, TextureUsage::TransferSrc) &&
        !has(desc.usage, TextureUsage::TransferDst) &&
        !has(desc.usage, TextureUsage::Storage);
    if (desc.transient && renderTargetOnly) {
        td->setStorageMode(MTL::StorageModeMemoryless);
    } else if (msaa) {
        td->setStorageMode(MTL::StorageModePrivate);
    } else {
        td->setStorageMode(isDepthFormat(desc.format) ? MTL::StorageModePrivate : MTL::StorageModeShared);
    }

    MTL::TextureUsage usage = 0;
    if (has(desc.usage, TextureUsage::Sampled))     usage |= MTL::TextureUsageShaderRead;
    if (has(desc.usage, TextureUsage::ColorTarget)) usage |= MTL::TextureUsageRenderTarget;
    if (has(desc.usage, TextureUsage::DepthTarget)) usage |= MTL::TextureUsageRenderTarget;
    if (has(desc.usage, TextureUsage::Storage))     usage |= MTL::TextureUsageShaderWrite;
    td->setUsage(usage);

    MTL::Texture* tex = I.device->newTexture(td);
    td->release();

    if (!initialData.empty() && tex) {
        MTL::Region region = MTL::Region::Make2D(0, 0, desc.width, desc.height);
        tex->replaceRegion(region, 0, initialData.data(), desc.width * 4);  // assumes RGBA8
    }

    uint32_t idx = Impl::acquire(I.textures, I.freeTextures);
    I.textures[idx].texture = NS::TransferPtr(tex);
    I.textures[idx].alive   = true;
    return TextureHandle{ idx, I.textures[idx].generation };
}

void Device::destroy(TextureHandle h) {
    if (!h.valid() || h.index >= impl_->textures.size()) return;
    auto& s = impl_->textures[h.index];
    if (!s.alive || s.generation != h.generation) return;
    s.texture.reset();
    s.alive = false;
    ++s.generation;
    impl_->freeTextures.push_back(h.index);
}

RenderTargetHandle Device::createRenderTarget(TextureHandle tex) {
    Impl& I = *impl_;
    uint32_t idx = Impl::acquire(I.renderTargets, I.freeRenderTargets);
    I.renderTargets[idx].textureIndex = tex.index;
    I.renderTargets[idx].alive = true;
    return RenderTargetHandle{ idx, I.renderTargets[idx].generation };
}

void Device::destroy(RenderTargetHandle h) {
    if (!h.valid() || h.index >= impl_->renderTargets.size()) return;
    auto& s = impl_->renderTargets[h.index];
    if (!s.alive || s.generation != h.generation) return;
    s.alive = false;
    ++s.generation;
    impl_->freeRenderTargets.push_back(h.index);
}

void Device::readback(TextureHandle h, std::span<std::byte> out) {
    if (!h.valid() || h.index >= impl_->textures.size()) return;
    auto& s = impl_->textures[h.index];
    if (!s.alive) return;
    MTL::Texture* tex = s.texture.get();
    NS::UInteger w = tex->width(), ht = tex->height();
    MTL::Region region = MTL::Region::Make2D(0, 0, w, ht);
    tex->getBytes(out.data(), w * 4, region, 0);   // assumes RGBA8 (4 bytes/px)
}

// -----------------------------------------------------------------------------
// Shaders & pipelines
// -----------------------------------------------------------------------------
ShaderHandle Device::createShader(std::span<const std::byte> blob, ShaderStage stage) {
    Impl& I = *impl_;
    dispatch_data_t dd = dispatch_data_create(blob.data(), blob.size(), nullptr,
                                              DISPATCH_DATA_DESTRUCTOR_DEFAULT);
    NS::Error* err = nullptr;
    MTL::Library* lib = I.device->newLibrary(dd, &err);
    dispatch_release(dd);
    if (!lib) {
        if (err) std::fprintf(stderr, "createShader: %s\n", err->localizedDescription()->utf8String());
        return {};
    }

    // Pick the entry point whose function type matches the requested stage.
    MTL::Function* chosen = nullptr;
    NS::Array* names = lib->functionNames();
    for (NS::UInteger i = 0; i < names->count(); ++i) {
        auto* name = static_cast<NS::String*>(names->object(i));
        MTL::Function* fn = lib->newFunction(name);
        MTL::FunctionType ft = fn->functionType();
        const bool match =
            (stage == ShaderStage::Vertex   && ft == MTL::FunctionTypeVertex)   ||
            (stage == ShaderStage::Fragment && ft == MTL::FunctionTypeFragment) ||
            (stage == ShaderStage::Compute  && ft == MTL::FunctionTypeKernel);
        if (match) { chosen = fn; break; }
        fn->release();
    }
    if (!chosen) { lib->release(); return {}; }

    uint32_t idx = Impl::acquire(I.shaders, I.freeShaders);
    I.shaders[idx].library  = NS::TransferPtr(lib);
    I.shaders[idx].function = NS::TransferPtr(chosen);
    I.shaders[idx].alive    = true;
    return ShaderHandle{ idx, I.shaders[idx].generation };
}

void Device::destroy(ShaderHandle h) {
    if (!h.valid() || h.index >= impl_->shaders.size()) return;
    auto& s = impl_->shaders[h.index];
    if (!s.alive || s.generation != h.generation) return;
    s.function.reset();
    s.library.reset();
    s.alive = false;
    ++s.generation;
    impl_->freeShaders.push_back(h.index);
}

PipelineHandle Device::createGraphicsPipeline(const GraphicsPipelineDesc& desc) {
    Impl& I = *impl_;
    auto* pd = MTL::RenderPipelineDescriptor::alloc()->init();
    pd->setVertexFunction(I.shaders[desc.vertex.index].function.get());
    // Depth-only pipelines (e.g. shadow maps) have no fragment shader + no color attachments.
    if (desc.fragment.valid() && desc.fragment.index < I.shaders.size())
        pd->setFragmentFunction(I.shaders[desc.fragment.index].function.get());

    if (!desc.colorFormats.empty()) {
        pd->colorAttachments()->object(0)->setPixelFormat(toMTLPixelFormat(desc.colorFormats[0]));
    }
    if (desc.depthFormat != Format::Undefined) {
        pd->setDepthAttachmentPixelFormat(toMTLPixelFormat(desc.depthFormat));
    }
    pd->setRasterSampleCount(desc.sampleCount > 0 ? desc.sampleCount : 1);   // MSAA (1 = off)

    // Vertex descriptor from the backend-agnostic VertexLayout (single interleaved buffer at 0).
    if (!desc.vertexLayout.attributes.empty()) {
        auto* vd = MTL::VertexDescriptor::alloc()->init();
        for (const auto& attr : desc.vertexLayout.attributes) {
            auto* a = vd->attributes()->object(attr.location);
            a->setFormat(toMTLVertexFormat(attr.format));
            a->setOffset(attr.offset);
            a->setBufferIndex(kVertexBufferBase);
        }
        auto* layout = vd->layouts()->object(kVertexBufferBase);
        layout->setStride(desc.vertexLayout.stride);
        layout->setStepFunction(MTL::VertexStepFunctionPerVertex);
        pd->setVertexDescriptor(vd);
        vd->release();
    }

    NS::Error* err = nullptr;
    MTL::RenderPipelineState* state = I.device->newRenderPipelineState(pd, &err);
    pd->release();
    if (!state) {
        if (err) std::fprintf(stderr, "createGraphicsPipeline: %s\n", err->localizedDescription()->utf8String());
        return {};
    }

    // Depth-stencil state is a separate Metal object, set on the encoder at bind time.
    NS::SharedPtr<MTL::DepthStencilState> depthState;
    if (desc.depthFormat != Format::Undefined) {
        auto* dsd = MTL::DepthStencilDescriptor::alloc()->init();
        dsd->setDepthCompareFunction(toMTLCompare(desc.depth.op));
        dsd->setDepthWriteEnabled(desc.depth.write);
        depthState = NS::TransferPtr(I.device->newDepthStencilState(dsd));
        dsd->release();
    }

    uint32_t idx = Impl::acquire(I.pipelines, I.freePipelines);
    I.pipelines[idx].state     = NS::TransferPtr(state);
    I.pipelines[idx].depthState = depthState;
    I.pipelines[idx].topology  = toMTLPrimitive(desc.topology);
    I.pipelines[idx].cull      = (desc.raster.cull == CullMode::Back)  ? MTL::CullModeBack
                               : (desc.raster.cull == CullMode::Front) ? MTL::CullModeFront
                                                                       : MTL::CullModeNone;
    // The engine's convention is "front = counter-clockwise around the outward normal" (world space;
    // all primitives + loaded meshes follow it). The Metal winding maps 1:1. See
    // notes/investigations/realtime-rendering/2026-07-23-backface-culling-winding.md.
    I.pipelines[idx].winding   = (desc.raster.frontFace == FrontFace::CounterClockwise)
                               ? MTL::WindingCounterClockwise : MTL::WindingClockwise;
    I.pipelines[idx].alive     = true;
    return PipelineHandle{ idx, I.pipelines[idx].generation };
}

PipelineHandle Device::createComputePipeline(const ComputePipelineDesc& desc) {
    Impl& I = *impl_;
    if (!desc.compute.valid() || desc.compute.index >= I.shaders.size()) return {};
    MTL::Function* fn = I.shaders[desc.compute.index].function.get();
    if (!fn) return {};
    NS::Error* err = nullptr;
    MTL::ComputePipelineState* state = I.device->newComputePipelineState(fn, &err);
    if (!state) {
        if (err) std::fprintf(stderr, "createComputePipeline: %s\n", err->localizedDescription()->utf8String());
        return {};
    }
    uint32_t idx = Impl::acquire(I.pipelines, I.freePipelines);
    I.pipelines[idx].computeState = NS::TransferPtr(state);
    I.pipelines[idx].isCompute    = true;
    I.pipelines[idx].alive        = true;
    return PipelineHandle{ idx, I.pipelines[idx].generation };
}
SamplerHandle Device::createSampler(const SamplerDesc& desc) {
    Impl& I = *impl_;
    auto toFilter = [](Filter f) {
        return f == Filter::Nearest ? MTL::SamplerMinMagFilterNearest : MTL::SamplerMinMagFilterLinear;
    };
    auto toMip = [](MipmapMode m) {
        return m == MipmapMode::Nearest ? MTL::SamplerMipFilterNearest : MTL::SamplerMipFilterLinear;
    };
    auto toAddr = [](AddressMode a) {
        switch (a) {
            case AddressMode::Repeat:      return MTL::SamplerAddressModeRepeat;
            case AddressMode::ClampToEdge: return MTL::SamplerAddressModeClampToEdge;
            case AddressMode::MirrorRepeat:return MTL::SamplerAddressModeMirrorRepeat;
        }
        return MTL::SamplerAddressModeClampToEdge;
    };
    auto* sd = MTL::SamplerDescriptor::alloc()->init();
    sd->setMinFilter(toFilter(desc.minFilter));
    sd->setMagFilter(toFilter(desc.magFilter));
    sd->setMipFilter(toMip(desc.mipmap));
    sd->setSAddressMode(toAddr(desc.addressU));
    sd->setTAddressMode(toAddr(desc.addressV));
    sd->setRAddressMode(toAddr(desc.addressW));
    sd->setMaxAnisotropy(static_cast<NS::UInteger>(desc.maxAnisotropy < 1.0f ? 1.0f : desc.maxAnisotropy));
    MTL::SamplerState* s = I.device->newSamplerState(sd);
    sd->release();
    if (!s) return {};
    uint32_t idx = Impl::acquire(I.samplers, I.freeSamplers);
    I.samplers[idx].sampler = NS::TransferPtr(s);
    I.samplers[idx].alive   = true;
    return SamplerHandle{ idx, I.samplers[idx].generation };
}
void Device::destroy(SamplerHandle h) {
    if (!h.valid() || !impl_ || h.index >= impl_->samplers.size()) return;
    auto& s = impl_->samplers[h.index];
    if (!s.alive || s.generation != h.generation) return;
    s.sampler.reset();
    s.alive = false;
    ++s.generation;
    impl_->freeSamplers.push_back(h.index);
}
void Device::destroy(PipelineHandle h) {
    if (!h.valid() || h.index >= impl_->pipelines.size()) return;
    auto& s = impl_->pipelines[h.index];
    if (!s.alive || s.generation != h.generation) return;
    s.state.reset();
    s.alive = false;
    ++s.generation;
    impl_->freePipelines.push_back(h.index);
}

uint32_t Device::registerBindlessTexture(TextureHandle h) {
    Impl& I = *impl_;
    if (!h.valid() || h.index >= I.textures.size() || !I.textures[h.index].alive) return 0xFFFF'FFFFu;
    uint32_t slot;
    if (!I.freeBindlessSlots.empty()) {
        slot = I.freeBindlessSlots.back();
        I.freeBindlessSlots.pop_back();
    } else {
        if (I.bindlessSlots.size() >= kMaxBindlessTextures) return 0xFFFF'FFFFu;
        slot = static_cast<uint32_t>(I.bindlessSlots.size());
        I.bindlessSlots.push_back(0xFFFF'FFFFu);
    }
    I.bindlessSlots[slot] = h.index;
    return slot;
}

void Device::unregisterBindlessTexture(uint32_t index) {
    Impl& I = *impl_;
    if (index >= I.bindlessSlots.size() || I.bindlessSlots[index] == 0xFFFF'FFFFu) return;
    I.bindlessSlots[index] = 0xFFFF'FFFFu;
    I.freeBindlessSlots.push_back(index);
}

void Device::generateMipmaps(TextureHandle h) {
    Impl& I = *impl_;
    if (!h.valid() || h.index >= I.textures.size() || !I.textures[h.index].alive) return;
    MTL::Texture* tex = I.textures[h.index].texture.get();
    if (!tex || tex->mipmapLevelCount() <= 1) return;
    MTL::CommandBuffer* cmd = I.queue->commandBuffer();          // autoreleased
    MTL::BlitCommandEncoder* blit = cmd->blitCommandEncoder();   // autoreleased
    blit->generateMipmaps(tex);
    blit->endEncoding();
    cmd->commit();
    cmd->waitUntilCompleted();
}

// -----------------------------------------------------------------------------
// Frame lifecycle
// -----------------------------------------------------------------------------
FrameContext Device::beginFrame() {
    FrameContext f;
    f.impl_ = std::make_unique<FrameContext::Impl>();
    // Throttle to framesInFlight: block until the frame slot we're about to reuse has completed
    // on the GPU. Signaled from the command buffer's completion handler in endFrame.
    if (impl_->frameSem) dispatch_semaphore_wait(impl_->frameSem, DISPATCH_TIME_FOREVER);
    const uint32_t fif = std::max(impl_->config.framesInFlight, 1u);
    f.impl_->frameIndex = static_cast<uint32_t>(impl_->frameCounter % fif);
    ++impl_->frameCounter;
    f.impl_->pool = NS::AutoreleasePool::alloc()->init();
    f.impl_->cmd  = impl_->queue->commandBuffer();   // autoreleased into the pool

    if (impl_->windowed && impl_->layer) {
        CA::MetalDrawable* drawable = impl_->layer->nextDrawable();   // autoreleased; may block
        f.impl_->drawable = drawable;
        if (drawable) {
            // Point the reserved swapchain texture slot at this frame's drawable texture.
            impl_->textures[impl_->swapTexSlot].texture = NS::RetainPtr(drawable->texture());
            f.impl_->swapchainRT = RenderTargetHandle{ impl_->swapRTSlot,
                                                       impl_->renderTargets[impl_->swapRTSlot].generation };
        }
    }
    return f;
}

CommandList Device::commandList(FrameContext& f) {
    f.impl_->cmdState.dev = impl_.get();
    f.impl_->cmdState.cmd = f.impl_->cmd;
    CommandList cl;
    cl.impl_ = &f.impl_->cmdState;
    return cl;
}

void Device::submit(FrameContext&, CommandList&) { /* Metal: recorded on the frame's cmd buffer */ }

void Device::endFrame(FrameContext&& f) {
    if (!f.impl_) return;
    // Signal the frames-in-flight semaphore once the GPU finishes this frame's work, freeing the
    // slot for reuse framesInFlight frames later.
    if (f.impl_->cmd && impl_->frameSem) {
        dispatch_semaphore_t sem = impl_->frameSem;
        f.impl_->cmd->addCompletedHandler([sem](MTL::CommandBuffer*) {
            dispatch_semaphore_signal(sem);
        });
    }
    if (impl_->windowed && f.impl_->drawable) {
        f.impl_->cmd->presentDrawable(f.impl_->drawable);
        f.impl_->cmd->commit();                 // let it pipeline; CAMetalLayer paces frames
    } else if (f.impl_->cmd) {
        f.impl_->cmd->commit();
        f.impl_->cmd->waitUntilCompleted();     // headless: block so readback is valid
    }
    if (f.impl_->pool) { f.impl_->pool->release(); f.impl_->pool = nullptr; }
}

void Device::waitIdle() {
    // Block until all in-flight frames complete (headless already blocks per-frame; windowed frames
    // signal asynchronously). Drains + restores the frames-in-flight semaphore.
    if (impl_) impl_->drainFramesInFlight();
}

uint32_t Device::framesInFlight() const { return std::max(impl_->config.framesInFlight, 1u); }

Swapchain* Device::swapchain() { return nullptr; }

// -----------------------------------------------------------------------------
// CommandList recording
// -----------------------------------------------------------------------------
void CommandList::beginRendering(const RenderTargetDesc& desc) {
    Device::Impl* dev = impl_->dev;
    auto* rp = MTL::RenderPassDescriptor::renderPassDescriptor();   // autoreleased

    if (!desc.color.empty()) {
        const ColorAttachment& c = desc.color[0];
        uint32_t texIndex = dev->renderTargets[c.target.index].textureIndex;
        auto* ca = rp->colorAttachments()->object(0);
        ca->setTexture(dev->textures[texIndex].texture.get());
        ca->setLoadAction(toMTLLoad(c.load));
        // MSAA resolve: if a resolve target is set, resolve the multisampled attachment into it
        // (on Apple TBDR this happens on-tile). We only need the resolved image, so resolve-only.
        if (c.resolveTarget.valid()) {
            uint32_t rIndex = dev->renderTargets[c.resolveTarget.index].textureIndex;
            ca->setResolveTexture(dev->textures[rIndex].texture.get());
            ca->setStoreAction(c.store == StoreOp::Store ? MTL::StoreActionStoreAndMultisampleResolve
                                                         : MTL::StoreActionMultisampleResolve);
        } else {
            ca->setStoreAction(toMTLStore(c.store));
        }
        ca->setClearColor(MTL::ClearColor::Make(c.clearColor[0], c.clearColor[1],
                                                c.clearColor[2], c.clearColor[3]));
    }
    if (desc.depth) {
        uint32_t texIndex = dev->renderTargets[desc.depth->target.index].textureIndex;
        auto* da = rp->depthAttachment();
        da->setTexture(dev->textures[texIndex].texture.get());
        da->setLoadAction(toMTLLoad(desc.depth->load));
        da->setStoreAction(toMTLStore(desc.depth->store));
        da->setClearDepth(desc.depth->clearDepth);
    }
    impl_->encoder = impl_->cmd->renderCommandEncoder(rp);   // autoreleased
}

void CommandList::endRendering() {
    if (impl_->encoder) { impl_->encoder->endEncoding(); impl_->encoder = nullptr; }
}

void CommandList::resourceBarrier(std::span<const ResourceTransition>) {
    // Metal tracks hazards automatically for tracked resources within a command buffer: a later
    // encoder that reads a resource an earlier encoder wrote is synchronized by the driver. So
    // there is nothing to do here for the default (tracked) path. The call exists so the render
    // graph can emit explicit transitions that the Vulkan backend WILL need (image-layout
    // transitions + pipeline barriers). If we later adopt untracked/heap/argument-buffer
    // resources, this is where MTL::Fence / useResource residency calls would go.
}

void CommandList::beginCompute() {
    impl_->computeEncoder = impl_->cmd->computeCommandEncoder();   // autoreleased
}

void CommandList::endCompute() {
    if (impl_->computeEncoder) { impl_->computeEncoder->endEncoding(); impl_->computeEncoder = nullptr; }
}

void CommandList::bindPipeline(PipelineHandle h) {
    auto& slot = impl_->dev->pipelines[h.index];
    if (impl_->computeEncoder) {
        if (slot.computeState) impl_->computeEncoder->setComputePipelineState(slot.computeState.get());
        return;
    }
    impl_->encoder->setRenderPipelineState(slot.state.get());
    if (slot.depthState) impl_->encoder->setDepthStencilState(slot.depthState.get());
    impl_->encoder->setCullMode(slot.cull);
    impl_->encoder->setFrontFacingWinding(slot.winding);
    impl_->topology = slot.topology;
}

void CommandList::bindVertexBuffer(BufferHandle b, uint32_t slot) {
    impl_->encoder->setVertexBuffer(impl_->dev->buffers[b.index].buffer.get(), 0,
                                    kVertexBufferBase + slot);
}

void CommandList::bindIndexBuffer(BufferHandle b, IndexType t) {
    impl_->indexBuffer = impl_->dev->buffers[b.index].buffer.get();
    impl_->indexType   = toMTLIndexType(t);
}
void CommandList::bindResources(const ResourceBindings& bindings) {
    // Bind resource buffers at their (low) Metal indices. In a compute scope, bind to the compute
    // encoder; otherwise to both vertex and fragment stages (Slang declares the uniform in both).
    // Bindless textures/samplers: TODO.
    if (impl_->computeEncoder) {
        for (const auto& b : bindings.buffers) {
            MTL::Buffer* buf = impl_->dev->buffers[b.buffer.index].buffer.get();
            impl_->computeEncoder->setBuffer(buf, b.offset, b.binding);
        }
        return;
    }
    for (const auto& b : bindings.buffers) {
        MTL::Buffer* buf = impl_->dev->buffers[b.buffer.index].buffer.get();
        impl_->encoder->setVertexBuffer(buf, b.offset, b.binding);
        impl_->encoder->setFragmentBuffer(buf, b.offset, b.binding);
    }
    // Explicitly-bound sampled textures + samplers (fragment stage) — e.g. tone-mapping the HDR
    // target. Bindless material textures are a separate (future) path.
    for (const auto& t : bindings.textures) {
        if (t.texture.index < impl_->dev->textures.size())
            impl_->encoder->setFragmentTexture(impl_->dev->textures[t.texture.index].texture.get(), t.binding);
    }
    for (const auto& s : bindings.samplers) {
        if (s.sampler.index < impl_->dev->samplers.size())
            impl_->encoder->setFragmentSamplerState(impl_->dev->samplers[s.sampler.index].sampler.get(), s.binding);
    }
}
void CommandList::setPushConstants(std::span<const std::byte>) { /* TODO */ }

void CommandList::bindBindlessTextures(uint32_t baseSlot) {
    // Bind every alive bindless-table entry to the fragment stage at baseSlot + slot. Only valid
    // inside a rendering scope (material sampling is a fragment-stage concern).
    if (!impl_->encoder) return;
    Device::Impl* dev = impl_->dev;
    for (uint32_t slot = 0; slot < dev->bindlessSlots.size(); ++slot) {
        const uint32_t texIndex = dev->bindlessSlots[slot];
        if (texIndex == 0xFFFF'FFFFu || texIndex >= dev->textures.size()) continue;
        if (!dev->textures[texIndex].alive) continue;
        impl_->encoder->setFragmentTexture(dev->textures[texIndex].texture.get(), baseSlot + slot);
    }
}

void CommandList::setViewport(float x, float y, float width, float height,
                              float minDepth, float maxDepth) {
    MTL::Viewport vp{ x, y, width, height, minDepth, maxDepth };
    impl_->encoder->setViewport(vp);
}

void CommandList::setScissor(int32_t x, int32_t y, uint32_t width, uint32_t height) {
    MTL::ScissorRect sr{ static_cast<NS::UInteger>(x), static_cast<NS::UInteger>(y), width, height };
    impl_->encoder->setScissorRect(sr);
}

void CommandList::draw(uint32_t vertexCount, uint32_t instanceCount,
                       uint32_t firstVertex, uint32_t firstInstance) {
    impl_->encoder->drawPrimitives(impl_->topology,
                                   static_cast<NS::UInteger>(firstVertex),
                                   static_cast<NS::UInteger>(vertexCount),
                                   static_cast<NS::UInteger>(instanceCount),
                                   static_cast<NS::UInteger>(firstInstance));
}

void CommandList::drawIndexed(uint32_t indexCount, uint32_t instanceCount,
                             uint32_t firstIndex, int32_t vertexOffset, uint32_t firstInstance) {
    const NS::UInteger indexStride = (impl_->indexType == MTL::IndexTypeUInt16) ? 2 : 4;
    impl_->encoder->drawIndexedPrimitives(
        impl_->topology,
        static_cast<NS::UInteger>(indexCount),
        impl_->indexType,
        impl_->indexBuffer,
        static_cast<NS::UInteger>(firstIndex) * indexStride,
        static_cast<NS::UInteger>(instanceCount),
        static_cast<NS::Integer>(vertexOffset),
        static_cast<NS::UInteger>(firstInstance));
}
void CommandList::drawIndexedIndirect(BufferHandle, uint64_t, uint32_t, uint32_t) { /* TODO */ }
void CommandList::dispatch(uint32_t groupsX, uint32_t groupsY, uint32_t groupsZ) {
    if (!impl_->computeEncoder) return;
    // RHI convention: dispatch() takes THREADGROUP counts; kernels use 64-wide 1D threadgroups
    // ([numthreads(64,1,1)]). The caller passes ceil(N/64) as groupsX.
    MTL::Size groups{ groupsX, groupsY, groupsZ };
    MTL::Size threadsPerGroup{ 64, 1, 1 };
    impl_->computeEncoder->dispatchThreadgroups(groups, threadsPerGroup);
}

} // namespace engine::rhi
