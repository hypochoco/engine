# 2026-07-02 — Graphics Interface (RHI) Plan

The actual interface design for the graphics layer, worked **backward from the ECS, cache
behavior, the milestone scene, and headless operation**. Builds on — does not repeat — the
two prior graphics investigations:

- [2026-07-01-graphics-refactor.md](2026-07-01-graphics-refactor.md): why the `Graphics` god
  class must be decomposed; the `PipelineBuilder` idea; ownership decomposition; render-list
  direction. Read §2–§3 for the problems this plan answers.
- [2026-07-02-metal-backend.md](2026-07-02-metal-backend.md): "add Metal == do the refactor";
  the Vulkan↔Metal concept table (§4); shaders are the biggest cost (§5); metal-cpp specifics
  (§6). This plan does **not** re-derive the concept mapping — see that table.

The current Vulkan code is committed + pushed, so we can delete/move freely. This is a plan;
no RHI code is written yet.

---

## 0. TL;DR (the decisions this plan proposes)

1. **Three layers**: `rhi` (thin hardware abstraction) → `render` (mid-level renderer: owns
   pipelines/passes/per-frame buffers, consumes render lists) → ECS extraction (produces
   render lists). `core` supplies geometry/data. Scene data flows *into* the renderer.
2. **Compile-time backend selection, no runtime vtables.** One `rhi::Device` type; the impl
   is a per-backend `.cpp`/`.mm` chosen by CMake. One backend per binary.
3. **Handle-based resources.** The `Device` owns all GPU objects in pools; the public API
   passes small value-type generational handles (`BufferHandle`, `TextureHandle`, …). This is
   the first real consumer of the deferred `memory/` generational-handle pattern.
4. **Bindless / argument-buffer-first binding model** + **Vulkan dynamic rendering** to line
   up with Metal and to scale to ECS/massive-instance workloads without per-draw descriptor
   churn.
5. **Graphics is an optional consumer of the world.** The ECS + physics must step with **no
   `Device` at all** (headless training); rendering attaches as a separate consumer, possibly
   offline over recorded state ("simulate now, render later").
6. **Batched/instanced draw path** driven by SoA per-instance data, designed so an
   indirect/GPU-driven path can slot in later without changing the ECS-facing contract.

Open decisions collected in §12.

---

## 1. Requirements (backward from the goals)

From goals.md + the milestone, the interface must satisfy:

| Driver | Interface implication |
|---|---|
| Vulkan **and** Metal, one per build | Public surface leaks no `Vk*`/`MTL::`; backend chosen at compile time. |
| Headless is first-class (ML training, offline render) | `Device` builds with no window/surface; render target can be an offscreen texture; **and** the world can run with no Device at all. |
| ML throughput / many parallel envs | Coarse API (no per-draw virtual calls/allocations); batched instancing; indirect-draw-ready; compute pipelines + compute queue. |
| ~100k instances (massive scale) | Geometry stored once + per-instance SoA buffer; bindless textures; minimal state changes per frame. |
| ECS is the organizing model | Renderer owns **no** scene data; it consumes per-view render lists extracted from the ECS each frame. |
| Cache performance | Handles not pointers; flat SoA render/instance arrays; sort/batch by pipeline→mesh→material. |
| Deferred rendering (milestone capability) | Multiple render targets (MRT) + multi-pass (geometry→lighting) reading a texture written by a prior pass; a light frame-graph can sit on top later. |
| Determinism (ML) | Rendering is a pure function of world state → reproducible; extraction ordering is stable. |

---

## 2. Layering & ownership

```
Application (consumer; owns lifecycle + the update→extract→render loop)
        │  creates world, systems, Device (windowed|headless), Renderer
        ▼
ECS world  ──(physics writes Transforms)──►  components: Transform, MeshRef, MaterialRef, ...
        │
        │  render-extraction system (per view): cull → sort/batch → SoA instance data
        ▼
render::RenderView { view/proj, target, RenderItem[], InstanceData[] }   ◄── transient, per frame
        │
        ▼
render::Renderer     owns: pipelines, pass/target definitions, per-frame ring buffers
        │            owns NO scene data. Translates render lists into RHI commands.
        ▼
rhi::Device / CommandList / Pipeline / Buffer / Texture ...   owns: all GPU objects (pools)
        │            backend-agnostic surface; one impl per backend (compile-time)
        ▼
[ Vulkan backend ]  OR  [ Metal backend ]     (only one compiled)

engine::core (no GPU): MeshData/Vertex/Material/ModelData → uploaded into Device buffers.
```

Key ownership rules:
- **`rhi::Device`** owns every GPU resource (buffers, textures, samplers, pipelines, shader
  modules, the geometry arenas) in pools, and the deferred-deletion queue. One per process.
- **`render::Renderer`** owns pipeline definitions, render-target/pass definitions, and the
  per-frame instance/uniform ring buffers. Owns no world data.
- **ECS** owns scene state. Extraction produces *transient* render lists (arena-allocated per
  frame, discarded after submit).
- **`core`** owns CPU geometry; hands `MeshData`/`Image` to the Device for upload, gets
  handles back.
- **`Swapchain` + `WindowSurface`** are optional and only exist in windowed mode.

This is the ownership inversion the refactor doc called for (§2 problem 7): scene/render data
lives in the ECS, not the renderer.

---

## 3. Core design decisions

### 3.1 Compile-time backend, no runtime dispatch
One backend per binary (CMake already implies this). So the RHI is **concrete types with a
per-backend implementation**, not abstract interfaces:

- Public headers declare `class Device { ... std::unique_ptr<Impl> impl_; };` etc.
- `src/graphics/vulkan/device.cpp` **or** `src/graphics/metal/device.mm` defines `Impl` —
  CMake compiles exactly one.
- No vtables, no per-call indirection. `pimpl` is used only on **coarse** objects (Device,
  Swapchain, CommandList, Pipeline) where one heap indirection is irrelevant. Fine-grained
  objects (buffers/textures) are **handles**, not pimpl objects (§3.2), so there's no
  per-resource allocation.
- A compile define (`ENGINE_RHI_VULKAN` / `ENGINE_RHI_METAL`) + a `rhi::backendName()` for
  logging.

Rejected: runtime-virtual RHI (needless vtable cost + we never mix backends in one binary).

### 3.2 Handle-based resources (ownership in the Device)
The public API passes small value handles; the Device owns the real objects in pools:

```cpp
namespace engine::rhi {
template <class Tag> struct Handle {
    uint32_t index      = kInvalid;   // slot in the Device pool
    uint32_t generation = 0;          // guards against stale handles after free/reuse
    static constexpr uint32_t kInvalid = 0xFFFF'FFFF;
    bool valid() const { return index != kInvalid; }
    bool operator==(const Handle&) const = default;
};
struct BufferTag; struct TextureTag; struct SamplerTag;
struct PipelineTag; struct ShaderTag; struct RenderTargetTag;
using BufferHandle       = Handle<BufferTag>;
using TextureHandle      = Handle<TextureTag>;
using SamplerHandle      = Handle<SamplerTag>;
using PipelineHandle     = Handle<PipelineTag>;
using ShaderHandle       = Handle<ShaderTag>;
using RenderTargetHandle = Handle<RenderTargetTag>;
}
```

Why handles (not RAII objects passed around):
- **Cache/ECS friendly**: components store 8-byte handles, not pointers; render lists are flat
  arrays of handles; no pointer chasing across scattered allocations.
- **Stable identity** across create/destroy churn (streaming, 100k spawns) — the generation
  catches use-after-free cheaply.
- **Backend-agnostic** by construction: a handle carries no `Vk*`/`MTL::`.
- Ties directly to the geometry-scaling conclusion (central store + handles).

> This generational `Handle` **is** the deferred `core/memory/` type. The RHI is its first
> real consumer → build the minimal version here; promote to `core` only if a second consumer
> (ECS storage, geometry store) wants the identical thing. (Per the YAGNI rule in the core
> plan.)

Trade-off accepted: a handle indirection (pool lookup) on use. At our call granularity
(per-batch, not per-instance) this is negligible and the lookup is a contiguous array index.

### 3.3 Binding model: bindless / argument-buffer-first
Descriptors are where Vulkan and Metal differ most (metal-backend §4). Rather than the
current fixed `NUM_TEXTURES=16` combined-sampler array + per-draw push-constant material
range, design for:
- A **global bindless texture table** (Vulkan descriptor indexing / Metal argument buffer).
  Per-instance `materialIndex` indexes into a materials buffer; materials index into the
  bindless table. No per-draw descriptor rebinding, natural on Metal, scales to many
  textures.
- Per-frame **uniform** (view/proj/lights) and **storage** (instance data, materials) buffers
  bound once per view.
- Expressed in the RHI as a small `ResourceBindings` object per pass, not per draw.

This removes refactor problems §2.2/§2.3 (interface baked into pipeline) and is the
scalable/ECS-friendly path.

### 3.4 Vulkan dynamic rendering
Use `VK_KHR_dynamic_rendering` so there's no `VkRenderPass`/`VkFramebuffer` object graph —
render targets are specified at `beginRendering` time, mirroring Metal's
`MTL::RenderPassDescriptor`. Makes the two backends line up and kills the framebuffer
bookkeeping in the old code.

---

## 4. The RHI surface (sketch)

Backend-agnostic types only. Enums (Format, BufferUsage, MemoryMode, ShaderStage, Topology,
CullMode, BlendPreset, DepthState, LoadOp/StoreOp, IndexType) live in `rhi/types.h`.

```cpp
namespace engine::rhi {

struct BufferDesc  { uint64_t size; BufferUsage usage; MemoryMode memory; };
struct TextureDesc { uint32_t width, height, depth = 1, mips = 1, layers = 1;
                     Format format; TextureUsage usage; uint32_t sampleCount = 1; };

struct GraphicsPipelineDesc {          // the PipelineBuilder config from the refactor doc
    ShaderHandle vertex, fragment;
    VertexLayout vertexLayout;         // derived from core::Vertex (see §10)
    Topology     topology   = Topology::TriangleList;
    CullMode     cull       = CullMode::Back;
    FrontFace    frontFace  = FrontFace::CCW;
    BlendPreset  blend      = BlendPreset::Opaque;
    DepthState   depth      = { .test = true, .write = true, .op = CompareOp::Less };
    uint32_t     sampleCount = 1;
    std::span<const Format> colorTargets;   // for dynamic rendering
    Format       depthTarget = Format::Depth32;
    // resource interface (bindless table + uniform/storage slots + push range)
    BindingLayout bindings;
};
struct ComputePipelineDesc { ShaderHandle compute; BindingLayout bindings; };

class Device {
public:
    static Device createHeadless(const DeviceConfig&);
    static Device createWindowed(WindowSurface&, const DeviceConfig&);

    // resource creation → handles (Device owns the objects)
    BufferHandle   createBuffer (const BufferDesc&,  const void* data = nullptr);
    TextureHandle  createTexture(const TextureDesc&, const void* data = nullptr);
    SamplerHandle  createSampler(const SamplerDesc&);
    ShaderHandle   createShader (std::span<const std::byte> blob, ShaderStage);
    PipelineHandle createGraphicsPipeline(const GraphicsPipelineDesc&);
    PipelineHandle createComputePipeline (const ComputePipelineDesc&);

    void  updateBuffer(BufferHandle, uint64_t offset, std::span<const std::byte>);
    void* map(BufferHandle);            // for persistent-mapped ring buffers
    void  destroy(BufferHandle);        // fence-gated deferred deletion (same for others)

    // frame lifecycle
    FrameContext beginFrame();          // waits fence, acquires cmd buffer (+ swapchain image)
    CommandList  commandList(FrameContext&);
    void         submit(FrameContext&, CommandList&&);
    void         endFrame(FrameContext&&);   // present (windowed) OR nothing/readback (headless)

    Swapchain* swapchain();             // null in headless
private:
    struct Impl; std::unique_ptr<Impl> impl_;   // Vulkan or Metal (compile-time)
};

class CommandList {                     // coarse recording; maps to VkCommandBuffer / MTL encoders
public:
    void beginRendering(const RenderTargetDesc&);   // dynamic rendering / MTLRenderPassDescriptor
    void bindPipeline(PipelineHandle);
    void bindVertexBuffer(BufferHandle, uint32_t slot = 0);
    void bindIndexBuffer(BufferHandle, IndexType);
    void bindResources(const ResourceBindings&);    // uniforms + storage + bindless table
    void setPushConstants(std::span<const std::byte>);
    void draw       (uint32_t vtx, uint32_t inst, uint32_t firstVtx, uint32_t firstInst);
    void drawIndexed(uint32_t idx, uint32_t inst, uint32_t firstIdx, int32_t vtxOffset, uint32_t firstInst);
    void drawIndexedIndirect(BufferHandle args, uint64_t offset, uint32_t drawCount);  // GPU-driven path
    void dispatch(uint32_t x, uint32_t y, uint32_t z);
    void endRendering();
};
} // namespace engine::rhi
```

`FrameContext`/`Fence`/`Semaphore` abstract per-frame sync and fix the old bugs by design
(N frames in flight parameterized; per-image vs per-frame semaphore handled inside the
backend, not exposed).

---

## 5. Memory & lifetime

- **Allocation**: Vulkan side integrates an allocator (VMA is Vulkan-only) behind
  `createBuffer/Texture`; Metal uses storage modes (Shared/Managed/Private). Neither leaks
  through the RHI surface.
- **RAII inside the backend**: backend objects wrapped so pools clean up on Device destroy;
  Metal manual refcount via `NS::SharedPtr` + per-frame `NS::AutoreleasePool` (metal-backend
  §6).
- **Deferred-deletion queue** in the Device: `destroy(handle)` enqueues; actual free happens
  after the fence for the frame that last used it — required once resources outlive a frame
  or with async upload.
- **Geometry arenas**: the central geometry store (from the scaling note) is realized as one
  large vertex `BufferHandle` + one index `BufferHandle` with sub-allocated `Mesh` ranges;
  `MeshHandle` → range. Uploading a `core::MeshData` appends to the arenas and returns a
  `MeshHandle`.

---

## 6. Mid-level renderer & the ECS-facing contract

The renderer is the stable boundary the ECS talks to. Its inputs are plain data:

```cpp
namespace engine::render {

using MeshHandle     = /* range into the geometry arenas */;
using MaterialHandle = uint32_t;   // index into the materials storage buffer

struct RenderItem {                // one batchable unit (one bind + one (indexed) draw)
    MeshHandle mesh;
    PipelineHandle pipeline;       // resolved from material/pass
    uint32_t firstInstance;
    uint32_t instanceCount;
};

struct InstanceData {              // SoA record uploaded to the per-frame instance buffer
    glm::mat4 model;
    glm::mat4 normalModel;         // (or packed mat3) — for correct normals under non-uniform scale
    uint32_t  materialIndex;
    uint32_t  _pad[3];
};

struct RenderView {
    glm::mat4 view, proj;
    RenderTargetHandle target;             // swapchain image OR offscreen texture(s)
    std::span<const RenderItem>   items;   // sorted by pipeline→mesh→material
    std::span<const InstanceData> instances;
};

class Renderer {
public:
    explicit Renderer(rhi::Device&);
    // Consumes render lists, uploads instance/uniform data, records + submits. Owns pipelines.
    void render(std::span<const RenderView> views);
};
} // namespace engine::render
```

The renderer:
1. uploads `instances` into a per-frame ring `BufferHandle` (storage buffer),
2. binds per-view uniforms (view/proj/lights) + the bindless table once,
3. walks `items`: `bindPipeline` (only on change) → `bindVertexBuffer`/`bindIndexBuffer`
   (the shared arenas, bound once) → `drawIndexed(instanceCount, firstInstance)`.

Because `items` is pre-sorted and instances are SoA, this is a tight, cache-friendly loop
with minimal state changes — and it's a straight swap to `drawIndexedIndirect` later without
touching the ECS contract.

---

## 7. ECS integration — walking the milestone

Components (in the future ECS, referencing render/core handles only — no `Vk*`):

```cpp
struct Transform    { glm::vec3 position; glm::quat rotation; glm::vec3 scale{1}; };
struct MeshRef      { render::MeshHandle mesh; };
struct MaterialRef  { render::MaterialHandle material; };
struct Renderable   {};   // tag: include in extraction
```

The **render-extraction system** (runs after physics, once per view):
```
for each entity with (Transform, MeshRef, MaterialRef, Renderable):
    if !frustum.contains(bounds(mesh) * transform):  continue      // uses core shape (deferred until now)
    key = (pipelineFor(material), mesh, material)
    append InstanceData{ modelMatrix(transform), normalMatrix, material.index } to a bucket[key]
sort buckets by key; concatenate → instances[] (SoA) + items[] (one RenderItem per contiguous run)
emit RenderView{ view, proj, target, items, instances }
```

### 7a. Ball rolling down a plane (the literal scene)
- Entities: `plane` (Transform, MeshRef=planeMesh, MaterialRef=groundMat), `ball`
  (Transform, MeshRef=sphereMesh, MaterialRef=ballMat).
- Physics integrates the ball's Transform each step (gravity + contact with the inclined
  plane). The renderer never sees physics — it reads Transforms.
- Extraction yields 2 RenderItems (plane×1 instance, sphere×1 instance), 2 InstanceData.
- One `RenderView` → 2 draws. Trivial, but exercises the whole path end to end.

### 7b. 100,000 spheres (the scale test)
- 1 plane + 100k ball entities sharing **one** `sphereMesh` and a small set of materials.
- Extraction: all spheres collapse into (sphere, mat) buckets → a handful of RenderItems,
  each with a large `instanceCount`. `instances[]` is 100k × `InstanceData` (~144 B) ≈ 14 MB
  written into the per-frame ring buffer.
- Draw: **one** `drawIndexed` with `instanceCount≈100k` per material bucket. Geometry stored
  once. This is the instancing story from the scaling note, realized through the RHI.
- Later: move culling/instance compaction to a compute pass + `drawIndexedIndirect` so the
  CPU doesn't touch 100k transforms — the contract (RenderView) is unchanged.

Sequence per frame:
```
physics.step(dt)                       // writes Transforms  (can run with NO Device)
for each view: extract(world,view) ──► RenderView
renderer.render(views) ──► Device: beginFrame → upload instances → record draws → endFrame
```

---

## 8. Headless & "simulate now, render later"

Three distinct headless needs, all supported by the layering:

1. **Pure simulation, no graphics** (ML training throughput): build the ECS world + physics
   with **no `Device`**. Graphics is an optional consumer; nothing in `core`/ECS/physics
   links the RHI. This is the most important headless mode and it's free if we keep the
   dependency direction clean.
2. **Headless rendering** (offline images / pixel-based RL): `Device::createHeadless()` — no
   surface/swapchain; render target is an offscreen `Texture` (color [+ depth/MRT]);
   `endFrame` does readback (or keeps it on GPU for a compute consumer) instead of present.
   Windowed vs headless differ *only* in the frame tail.
3. **Simulate now, render later**: because rendering is a pure function of world state,
   record trajectories (component snapshots) during a fast headless sim, then replay them
   through extraction + a (possibly higher-quality, non-realtime) renderer offline. The RHI
   doesn't need to know; the renderer just consumes RenderViews built from replayed state.

**Deferred rendering** (the milestone capability) sits on top of the same primitives:
geometry pass writes an MRT G-buffer (albedo/normal/depth) via `beginRendering` with multiple
color targets; a lighting pass binds those textures and writes the final target. Two
`RenderView`s / passes over the same instance data. This is why MRT + reading a
prior-pass texture must be first-class, and why a light **frame-graph** may later sit above
the renderer (don't build it yet; keep passes cheap + data-driven).

**Parallel environments**: one `Device` per process; each env is its own ECS world. Training
that needs pixels renders each world to its own offscreen target (or a texture-array/atlas)
and submits batched. Envs that don't render never touch the Device. Determinism holds because
render output is a function of world state and extraction order is stable.

---

## 9. Backend switching mechanics

File layout:
```
include/engine/graphics/rhi/   # backend-agnostic public headers (types, handles, Device, ...)
include/engine/graphics/render/# Renderer + RenderView/RenderItem/InstanceData
src/graphics/
├── common/                    # backend-independent .cpp (Renderer, geometry store, ...)
├── vulkan/                    # compiled when NOT Apple
└── metal/                     # compiled when Apple (+ the .mm CAMetalLayer/window shim)
```
CMake (folds in metal-backend §6): enable `OBJCXX` on Apple; link Foundation+Metal+QuartzCore;
add `external/metal-cpp` include (Apple); **per-backend source selection** (glob the chosen
backend dir, not one recursive glob); one TU defines the metal-cpp impl macros; define
`ENGINE_RHI_VULKAN`/`ENGINE_RHI_METAL`.

---

## 10. `core` ↔ graphics boundary

- `core::Vertex` stays pure data. Each backend derives its vertex layout: a
  `vulkan/vertex_format.cpp` (binding/attribute descriptions) and `metal/vertex_format.mm`
  (`MTLVertexDescriptor`), both from the single `core::Vertex` field list. The RHI's
  `VertexLayout` is the backend-agnostic description the renderer passes to
  `GraphicsPipelineDesc`.
- `core::MeshData` → `Device` geometry arenas → `MeshHandle`. `core::Image` (when we
  un-defer it) → `Device::createTexture` → `TextureHandle`.
- `core::Material` (base color + texture ref) → an entry in the materials storage buffer;
  the texture ref resolves to a slot in the bindless table.
- Direction stays one-way: graphics depends on core; core never depends on graphics/RHI.

## 11. Performance levers (summary)

Compile-time backend (no vtables) · handles not pointers (cache, stable identity) · geometry
stored once + SoA instance ring buffer · bindless textures (no per-draw descriptor churn) ·
pre-sorted render lists → minimal state changes · N frames in flight (fix the `=1` stall +
per-image semaphore bug by design) · persistent-mapped ring buffers · indirect-draw-ready
contract · compute queue for culling/instance compaction later.

---

## 12. Decisions (owner, 2026-07-02)

> **DECIDED**: (1) **handle-based** RHI. (2) **bindless** from the start. (3) **Metal/Apple
> first** — Vulkan code becomes a reference reorganized behind the RHI later. (4) **Slang**
> for shaders (one source → SPIR-V + MSL). Details below kept for rationale.

1. **RHI object style**: handle-based (recommended, §3.2) vs move-only RAII objects passed
   around? (Affects every signature.) → **handles**.
2. **Binding model**: go bindless/argument-buffer from the start (recommended, §3.3) vs a
   simpler fixed descriptor set first, bindless later? → **bindless from the start**.
3. **Which backend first?** Dev machine is Apple → **Metal first** is the only *runnable*
   path here; the existing Vulkan code becomes a reference we reorganize behind the RHI
   (compile-checked on Linux/CI). Alternative: Vulkan reorg first (reuses working code) but
   can't run/verify on this machine. → **Metal first**.
4. **Shader toolchain** (still open from metal-backend §5): Slang (one source → SPIR-V + MSL,
   recommended) vs GLSL + SPIRV-Cross. Blocks any real pipeline. → **Slang**.

Still open (not blocking header work):
5. **Metal API level**: classic `MTL` (widest support) vs `MTL4` (argument tables, closer to
   Vulkan; vendored). Affects the binding-model impl.
6. **Indirect / GPU-driven now or later?** Recommend: define the RenderView contract now,
   implement CPU instancing first, add indirect behind the same contract later.
7. **Frame-graph**: hand-rolled passes first; introduce a declarative graph only when
   deferred + offscreen + multi-view passes justify it.

---

## 13. Suggested sequencing (interface first)

0. Decide §12.1–§12.4 (they shape the headers).
1. **Write the `rhi` headers** (types/handles/descs/Device/CommandList) — no impl. Compiles
   everywhere; nothing links a backend yet.
2. **Write the `render` headers** (RenderView/RenderItem/InstanceData/Renderer) + the
   geometry-store/`MeshHandle` design against `core`.
3. **Stand up one backend** behind the RHI to first-triangle **offscreen (headless)** with a
   `tst/` driver that reads back a pixel — no window needed, provable on this machine (⇒
   Metal first, per §12.3).
4. Add the **windowed** tail (Swapchain + `.mm` CAMetalLayer shim) once headless works.
5. Reorganize the **Vulkan** code behind the same RHI (CI/Linux verified).
6. Wire the **ECS extraction** path and drive the milestone (ball on plane → 100k spheres).

Each step keeps the build green and is independently reviewable.
