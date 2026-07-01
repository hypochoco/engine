# Architecture (as-is)

Snapshot of the current structure. This describes what exists today, not the target.
Last synced with code: 2026-07-01.

## Build & layout

- C++23, CMake `>= 3.25`, `compile_commands.json` exported.
- Split `include/` (public headers) vs `src/` (implementation).
- Per-module build files live in top-level `graphics/` and `physics/` dirs, separate from
  their source under `src/`.

```
engine/
├── CMakeLists.txt          # top-level: deps + module wiring
├── include/engine/
│   ├── graphics/graphics.h # the entire graphics interface
│   └── physics/physics.h   # stub
├── src/
│   ├── graphics/           # 5 .cpp files, ~115 KB total
│   └── physics/physics.cpp # stub
├── graphics/CMakeLists.txt # defines engine_graphics (STATIC)
├── physics/CMakeLists.txt  # defines engine_physics (STATIC)
└── external/               # submodules: glfw, stb, tinyobjloader
```

## Target graph

```
engine::engine (INTERFACE)
├── engine::graphics (STATIC) → glfw, glm::glm, Vulkan::Vulkan, tinyobjloader, stb::stb
└── engine::physics  (STATIC)
```

Dependencies: `glm` and `Vulkan` via `find_package`; `glfw` + `tinyobjloader` via
`add_subdirectory`; `stb` as a header-only INTERFACE lib.

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
- No headless/offscreen *device* path; rendering assumes a GLFW window + swapchain.
- No compute pipelines or compute-queue usage.
- No shaders in-tree and no SPIR-V compile step in CMake (pipelines take shader paths at
  runtime).
- No tests.
