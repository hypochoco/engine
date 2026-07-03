//
//  resources.h
//  engine::graphics / rhi
//
//  Descriptors for creating buffers, textures, samplers, and describing vertex input.
//  Pure data passed to Device::create*.
//

#pragma once

#include <cstdint>
#include <vector>

#include "engine/graphics/rhi/types.h"

namespace engine::rhi {

struct BufferDesc {
    uint64_t    size   = 0;
    BufferUsage usage  = BufferUsage::None;
    MemoryMode  memory = MemoryMode::GpuOnly;
    const char* debugName = nullptr;
};

struct TextureDesc {
    uint32_t     width  = 1;
    uint32_t     height = 1;
    uint32_t     depth  = 1;
    uint32_t     mipLevels   = 1;
    uint32_t     arrayLayers = 1;
    uint32_t     sampleCount = 1;
    Format       format = Format::RGBA8Unorm;
    TextureUsage usage  = TextureUsage::Sampled;
    const char*  debugName = nullptr;
};

enum class Filter      { Nearest, Linear };
enum class MipmapMode  { Nearest, Linear };
enum class AddressMode { Repeat, ClampToEdge, MirrorRepeat };

struct SamplerDesc {
    Filter      minFilter = Filter::Linear;
    Filter      magFilter = Filter::Linear;
    MipmapMode  mipmap    = MipmapMode::Linear;
    AddressMode addressU  = AddressMode::Repeat;
    AddressMode addressV  = AddressMode::Repeat;
    AddressMode addressW  = AddressMode::Repeat;
    float       maxAnisotropy = 1.0f;
};

// -----------------------------------------------------------------------------
// Vertex input
// -----------------------------------------------------------------------------
// Backend-agnostic description of interleaved vertex attributes. Derived from
// core::Vertex by the render layer (render::coreVertexLayout()) — rhi does not depend
// on core.
enum class VertexFormat { Float, Float2, Float3, Float4, UByte4Norm };

struct VertexAttribute {
    uint32_t     location = 0;   // shader input location
    VertexFormat format   = VertexFormat::Float3;
    uint32_t     offset   = 0;   // byte offset within the vertex
};

struct VertexLayout {
    uint32_t                     stride = 0;   // bytes per vertex
    std::vector<VertexAttribute> attributes;
};

} // namespace engine::rhi
