//
//  types.h
//  engine::graphics / rhi
//
//  Foundational, backend-agnostic RHI types: generational resource handles and the
//  enums used across descriptors and commands. No Vk*/MTL:: types appear here or in any
//  rhi header — the backend is chosen at compile time and implemented in
//  src/graphics/{metal,vulkan}/.
//

#pragma once

#include <cstdint>

#include "engine/core/memory/handle.h"

namespace engine::rhi {

// -----------------------------------------------------------------------------
// Handles
// -----------------------------------------------------------------------------
// The Device owns every GPU object in pools; the public API passes these small value
// handles. `generation` guards against using a handle whose slot was freed and reused.
// The handle type is shared engine-wide (see core/memory/handle.h).
template <class Tag>
using Handle = engine::core::Handle<Tag>;

struct BufferTag;
struct TextureTag;
struct SamplerTag;
struct ShaderTag;
struct PipelineTag;
struct RenderTargetTag;

using BufferHandle       = Handle<BufferTag>;
using TextureHandle      = Handle<TextureTag>;
using SamplerHandle      = Handle<SamplerTag>;
using ShaderHandle       = Handle<ShaderTag>;
using PipelineHandle     = Handle<PipelineTag>;
using RenderTargetHandle = Handle<RenderTargetTag>;

// -----------------------------------------------------------------------------
// Shader stages (bitmask)
// -----------------------------------------------------------------------------
enum class ShaderStage : uint32_t {
    None     = 0,
    Vertex   = 1u << 0,
    Fragment = 1u << 1,
    Compute  = 1u << 2,
};

constexpr ShaderStage operator|(ShaderStage a, ShaderStage b) {
    return static_cast<ShaderStage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr ShaderStage operator&(ShaderStage a, ShaderStage b) {
    return static_cast<ShaderStage>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
constexpr bool any(ShaderStage s) { return static_cast<uint32_t>(s) != 0; }

// -----------------------------------------------------------------------------
// Formats & fixed-function enums
// -----------------------------------------------------------------------------
enum class Format {
    Undefined,
    // color
    R8Unorm,
    RG8Unorm,
    RGBA8Unorm,
    RGBA8Srgb,
    BGRA8Unorm,
    RGBA16Float,
    RGBA32Float,
    // depth / stencil
    Depth32Float,
    Depth24Stencil8,
};

enum class IndexType { Uint16, Uint32 };

enum class Topology { TriangleList, TriangleStrip, LineList, PointList };

enum class CullMode { None, Front, Back };

enum class FrontFace { CounterClockwise, Clockwise };

enum class PolygonMode { Fill, Line };

enum class CompareOp { Never, Less, Equal, LessEqual, Greater, NotEqual, GreaterEqual, Always };

// Common blend presets — expands into full blend state in the backend.
enum class BlendPreset { Opaque, AlphaBlend, Additive, PremultipliedAlpha };

// Attachment load/store behavior at pass boundaries (matches Metal; maps to VK dynamic rendering).
enum class LoadOp  { Load, Clear, DontCare };
enum class StoreOp { Store, DontCare };

// -----------------------------------------------------------------------------
// Resource usage / memory
// -----------------------------------------------------------------------------
enum class BufferUsage : uint32_t {
    None          = 0,
    Vertex        = 1u << 0,
    Index         = 1u << 1,
    Uniform       = 1u << 2,
    Storage       = 1u << 3,
    Indirect      = 1u << 4,
    TransferSrc   = 1u << 5,
    TransferDst   = 1u << 6,
};
constexpr BufferUsage operator|(BufferUsage a, BufferUsage b) {
    return static_cast<BufferUsage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr BufferUsage operator&(BufferUsage a, BufferUsage b) {
    return static_cast<BufferUsage>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

enum class TextureUsage : uint32_t {
    None           = 0,
    Sampled        = 1u << 0,
    ColorTarget    = 1u << 1,
    DepthTarget    = 1u << 2,
    Storage        = 1u << 3,
    TransferSrc    = 1u << 4,
    TransferDst    = 1u << 5,
};
constexpr TextureUsage operator|(TextureUsage a, TextureUsage b) {
    return static_cast<TextureUsage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr TextureUsage operator&(TextureUsage a, TextureUsage b) {
    return static_cast<TextureUsage>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

// Where a resource's memory lives / how the CPU accesses it.
//  - GpuOnly:      device-local, no CPU access (Vulkan DEVICE_LOCAL / Metal Private).
//  - CpuToGpu:     host-visible, mapped, for per-frame upload (Metal Shared / VK host-visible).
//  - GpuToCpu:     readback (headless render → CPU).
enum class MemoryMode { GpuOnly, CpuToGpu, GpuToCpu };

} // namespace engine::rhi
