//
//  render_view.h
//  engine::graphics / view
//
//  The neutral, head-agnostic scene contract. An extraction system turns world state into these
//  plain-data render lists each frame; a renderer head (realtime OR path tracer) consumes them.
//  No Vk*/MTL::, no ECS types, and nothing renderer-specific (no pipelines) — just "what's in the
//  scene." Lives in graphics/view/ (not render/) so both heads depend on it without pulling either
//  renderer.
//

#pragma once

#include <cstdint>
#include <span>

#include <glm/glm.hpp>

#include "engine/graphics/rhi/types.h"

namespace engine::render {

// Handle into the GeometryStore's shared vertex/index arenas.
struct MeshTag;
using MeshHandle = rhi::Handle<MeshTag>;

// Index into the per-frame materials storage buffer.
using MaterialHandle = uint32_t;

// One batchable unit: a contiguous run of instances sharing a mesh + material. Becomes one
// (indexed, instanced) draw. Head-neutral: how it is drawn (raster pipeline, or accel-structure
// hit shading) is the consuming renderer's concern, not part of this scene contract.
struct RenderItem {
    MeshHandle mesh;
    uint32_t   firstInstance = 0;
    uint32_t   instanceCount = 0;
    // Optional per-item pipeline override (e.g. a foliage/wind or terrain variant built via
    // Renderer::createMeshPipeline). Invalid ⇒ the renderer's default mesh pipeline. Lets opaque,
    // foliage, and other variants coexist in one view without the app touching backend code.
    rhi::PipelineHandle pipeline;
};

// Per-instance record, SoA-friendly, uploaded into the per-frame instance storage buffer.
// std140/std430-friendly layout (16-byte aligned; matrices then scalars + padding).
struct InstanceData {
    glm::mat4 model{1.0f};
    glm::mat4 normalModel{1.0f};   // for correct normals under non-uniform scale
    uint32_t  materialIndex = 0;
    uint32_t  _pad[3]       = {0, 0, 0};
};

// Material feature flags (MaterialGPU::flags bitfield). General, content-agnostic surface
// properties — domain effects like vegetation wind belong in game shaders, not the engine.
enum MaterialFlags : uint32_t {
    MaterialFlagNone        = 0u,
    MaterialFlagAlphaCutout = 1u << 0,   // discard fragments whose albedo alpha < alphaCutoff
    MaterialFlagDoubleSided = 1u << 1,   // two-sided LIGHTING: flip the normal to face the viewer on
                                         // back faces (pair with a CullMode::None pipeline for foliage)
};

// GPU material record (matches the shader's Material — metallic-roughness workflow). Indexed by
// InstanceData::materialIndex. All textures are bindless-table slots (-1 = none). Defaults describe
// a plain opaque Lambert surface (metallic 0, roughness 1, no emissive) so untextured materials
// render exactly as before.
struct MaterialGPU {
    glm::vec4 baseColorFactor{1.0f};             // rgba
    glm::vec4 emissiveFactor{0.0f};              // rgb (a unused)
    float     metallicFactor  = 0.0f;
    float     roughnessFactor = 1.0f;
    float     alphaCutoff     = 0.5f;
    uint32_t  flags           = MaterialFlagNone;
    int32_t   baseColorTexture         = -1;
    int32_t   normalTexture            = -1;
    int32_t   metallicRoughnessTexture = -1;     // glTF pack: G = roughness, B = metallic
    int32_t   emissiveTexture          = -1;
    int32_t   occlusionTexture         = -1;     // R channel
    int32_t   _pad[3]                  = {0, 0, 0};
};

// A directional (sun) light + ambient term for simple Lambert shading. Plain data set per
// view; the Renderer packs it into the global uniform the mesh shader reads.
struct DirectionalLight {
    glm::vec3 direction{-0.4f, -0.8f, -0.5f};   // direction the light travels (world space)
    float     intensity = 1.0f;
    glm::vec3 color{1.0f, 0.98f, 0.95f};        // light color (multiplied by intensity)
    glm::vec3 ambient{0.12f, 0.13f, 0.16f};     // flat ambient term
};

// A punctual point light. Contributes Lambert diffuse with smooth range-based attenuation.
// std430/16-byte-friendly layout (vec3+float, vec3+float) — matches the shader's PointLight.
struct PointLight {
    glm::vec3 position{0.0f};      // world-space position
    float     radius   = 5.0f;     // influence radius (attenuation → 0 at radius)
    glm::vec3 color{1.0f};         // light color
    float     intensity = 1.0f;    // scalar multiplier
};

// A single view/pass to render. Multiple views enable multi-pass (deferred G-buffer →
// lighting), multiple cameras, or many parallel-env targets.
struct RenderView {
    glm::mat4 view{1.0f};
    glm::mat4 proj{1.0f};
    rhi::RenderTargetHandle target;          // swapchain image OR offscreen texture(s)
    uint32_t  width  = 0;                     // target dimensions (viewport + depth sizing)
    uint32_t  height = 0;
    float     clearColor[4] = {0.08f, 0.10f, 0.14f, 1.0f};
    DirectionalLight light{};                 // world directional (sun) + ambient for this view

    // Opaque per-view app data, uploaded verbatim into the shader Globals (read as globals.appData).
    // The engine never interprets it — game shaders (foliage/water/custom variants) decide its
    // meaning (e.g. x = elapsed time; yzw = wind direction/strength). Keeps domain effects like
    // vegetation wind in game shaders, not the engine ("the game decides what it passes").
    glm::vec4 appData{0.0f};

    std::span<const RenderItem>   items;      // pre-sorted by mesh → material
    std::span<const InstanceData> instances;  // indexed by RenderItem instance ranges
    std::span<const MaterialGPU>  materials;  // indexed by InstanceData::materialIndex
    std::span<const PointLight>   pointLights; // punctual lights affecting this view
};

} // namespace engine::render
