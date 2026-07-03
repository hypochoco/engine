# Architecture (as-is)

Snapshot of the current structure. This describes what exists today, not the target.
Last synced with code: 2026-07-02.

## Build & layout

- C++23, CMake `>= 3.25`, `compile_commands.json` exported.
- Split `include/` (public headers) vs `src/` (implementation).
- Per-module build files live in top-level `core/`, `graphics/`, `physics/`, `tst/` dirs,
  separate from their source under `src/`. CMake uses `GLOB_RECURSE` (incl. `.mm`) +
  `source_group`.
- **Backend is platform-selected**: `if(APPLE)` → Metal + QuartzCore frameworks;
  `else()` → `find_package(Vulkan)`.

```
engine/
├── CMakeLists.txt          # top-level: deps + platform backend select + module wiring
├── include/engine/
│   ├── core/core.h         # NEW — empty stub (backend-agnostic data target)
│   ├── graphics/graphics.h # the entire graphics interface (still Vulkan-only)
│   └── physics/physics.h   # stub
├── src/
│   ├── core/core.cpp       # NEW — empty stub
│   ├── graphics/           # 5 .cpp files, ~115 KB total (all Vulkan)
│   └── physics/physics.cpp # stub
├── core/CMakeLists.txt     # NEW — engine_core (STATIC)
├── graphics/CMakeLists.txt # engine_graphics (STATIC); Metal|Vulkan link per platform
├── physics/CMakeLists.txt  # engine_physics (STATIC)
├── tst/                    # NEW — placeholder `test` executable
└── external/               # submodules: glfw, glm, stb, tinyobjloader, metal-cpp (NEW)
```

## Target graph

```
engine::engine (INTERFACE)
├── engine::core     (STATIC) → glm::glm, tinyobjloader, stb::stb           [empty so far]
├── engine::graphics (STATIC) → glfw, glm, tinyobjloader, stb,
│                                (Apple) Metal+QuartzCore | (else) Vulkan::Vulkan
└── engine::physics  (STATIC)
```

Dependencies: `glm` via `find_package`; `glfw` + `tinyobjloader` via `add_subdirectory`;
`stb` header-only INTERFACE; `metal-cpp` vendored (not yet wired into include dirs).

> ✅ **Build reality (2026-07-03)**: the full tree builds on macOS; the Metal backend renders
> core meshes both **offscreen and to a window**. `engine_graphics` = RHI + Metal backend
> (headless & windowed `MTL::Device`; handle pools; Slang `.metallib` shader libs; graphics
> pipeline + vertex descriptor + depth-stencil; render-encoder frame lifecycle; indexed draw
> + depth; readback; **CAMetalLayer swapchain + present** via an Objective-C++ shim
> `metal_window.mm`). `render::GeometryStore` uploads `core::MeshData` → shared buffers →
> `MeshHandle`s. Tests: `rhi_smoke`, `triangle_offscreen`, `mesh_offscreen` (headless,
> pixel-verified) and **`visual_window`** (opens a GLFW window, renders a lit `core` sphere in
> a present loop). CMake enables `OBJC`+`OBJCXX` on Apple, links Cocoa/Metal/QuartzCore/
> Foundation. Shaders via `slangc` (`shaders/{triangle,mesh}.slang`; toolchain from
> `scripts/get_slang.sh` → gitignored `tools/`). Legacy Vulkan parked under
> `src/graphics/vulkan/`. See investigations/2026-07-02-rhi-interface-plan.md §13.

## Graphics module

A single monolithic `Graphics` class (~460 lines of declarations in `graphics.h`),
implemented across five files. Structure is derived from the vulkan-tutorial.com flow.

- **`graphics.cpp`** (~48 KB) — instance/device selection, logical device, textures
  (staging, mipmaps, image views, sampler), buffers, single-time command helpers,
  memory-type lookup, debug messenger, validation layers, macOS portability handling.
- **`graphics_swapchain.cpp`** (~29 KB) — swapchain create/recreate/cleanup, render pass,
  descriptor set layout/pool/sets, the main graphics pipeline, command buffer recording,
  and the per-frame acquire → record → submit → present loop.
- **`graphics_custom.cpp`** (~27 KB) — reusable helpers for *custom/offscreen* targets:
  `createRenderPass`, `createFramebuffer`, `createDescriptorSet(s)`, `createPipeline`
  (incl. push-constant overload), and granular `record*` command helpers
  (begin/end render pass, viewport, scissor, clear, bind descriptor, push constant, draw).
- **`graphics_model.cpp`** (~8 KB) — `loadQuad`, `loadObj` (via tinyobjloader).
- **`graphics_instance.cpp`** (~2 KB) — `updateGlobalUBO`, `addDrawJob`,
  `copyInstanceToBuffer`.

### Data structures (in graphics.h)

- `Vertex` (pos/color/texCoord + binding/attribute descriptions + hash).
- `Mesh` (first/count for vertices and indices into shared buffers).
- `Material` (`textureIndex`).
- `GlobalUBO` (view, proj; lights TODO).
- `InstanceSSBO` (model matrix).
- `DrawJob` (visible flag, meshIndex, material range, instance range).
- Vulkan helpers: `SwapChainSupportDetails`, `QueueFamilyIndices`.

### Rendering model (seed of data-oriented design)

Shared vertex/index buffers hold all mesh data; `Mesh` records ranges into them.
`DrawJob`s reference a mesh + material range + instance range. Instance model matrices
live in a per-frame SSBO (`instanceStorageBuffers`, filled by `copyInstanceToBuffer`).
This is the closest thing to an ECS-friendly, batched render path — but nothing feeds it
from an entity layer yet.

### Hardcoded caps

`MAX_FRAMES_IN_FLIGHT = 1`, `NUM_TEXTURES = 16`, `MAX_ENTITIES = 16`, `WIDTH/HEIGHT 800x600`.

## Physics module

Stub only: `Physics::test()` prints `"hello physics"`. No simulation, no integrator,
no collision, no broadphase.

## What is NOT here yet

- No ECS (no entity/component/system/registry/archetype types anywhere).
- No application entry point (`main`) or engine loop driver — this is a library with no
  consumer.
- Backend abstraction: the RHI **interface** headers exist (`include/engine/graphics/rhi/`
  + `render/`, no `Vk*`/`MTL::`), and a **Metal backend skeleton** implements the headless
  `Device` + buffer pool. Still missing: pipelines, command recording, swapchain/present,
  shaders (Slang toolchain), and the whole Vulkan port behind the RHI (legacy code parked
  under `src/graphics/vulkan/`, not yet reorganized).
- `engine::core` exists but is empty (Vertex/Mesh/Material/loader not moved yet).
- No headless/offscreen *device* path; rendering assumes a GLFW window + swapchain.
- No compute pipelines or compute-queue usage.
- No shaders in-tree and no compile step (pipelines take shader paths at runtime; nothing
  emits SPIR-V, let alone Metal `.metallib`).
- Tests: only a placeholder `test` executable that prints a string.
