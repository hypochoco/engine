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

> ✅ **Build reality (2026-07-03)**: full tree builds on macOS; the Metal backend renders
> **instanced core meshes through the Renderer/RenderView path**, offscreen and to a window.
> `engine_graphics` = RHI + Metal backend (headless & windowed Device; handle pools; Slang
> `.metallib` libs; pipeline + vertex descriptor + depth-stencil; render-encoder lifecycle;
> **indexed + instanced draw**; depth; readback; CAMetalLayer present via `metal_window.mm`).
> `render::GeometryStore` uploads `core::MeshData` → shared buffers; **`render::Renderer`**
> consumes `RenderView`s (camera uniform @buffer0 + per-instance storage @buffer1 + materials
> storage @buffer2 + shared vertex data @buffer16), owning the per-frame buffers + depth
> target. **Per-instance materials**: each instance carries a `materialIndex` into a materials
> buffer (`baseColorFactor`), so instanced draws are individually colored (bindless *textures*
> are the reserved next step: `baseColorTexture` field + `Device::registerBindlessTexture`
> exist but aren't wired). Shader `shaders/mesh.slang` is instanced (SV_InstanceID → per-
> instance model + material). Tests:
> `rhi_smoke`, `triangle_offscreen`, `mesh_offscreen` (headless: 3 instanced spheres, each a
> different material color, pixel-verified red center), `visual_window` (windowed: NxN
> instanced sphere grid with a per-instance color gradient, orbiting camera). Shaders via
> `slangc` (column-major). Legacy Vulkan parked under
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

## Physics module (`engine::physics`, Phase 0, 2026-07-03)

ECS-free, backend-agnostic core (depends on `engine::core` only — no ecs, no graphics).
Design + phasing + differentiable-backend design-ahead: investigations/2026-07-03-physics-plan.md.
- **Shapes**: `Sphere`, `Plane` (half-space). GJK `support()` seam is Phase 2.
- **Collision** (`collide::sphereVsPlane`, `sphereVsSphere`): exact fast paths filling a
  **solver-agnostic** `Contact` (continuous signed `separation`, normal, point) — consumable
  by both a future impulse solver and a compliant/soft solver.
- **Dynamics**: `RigidBodyState` (position, orientation quat, lin/ang velocity, invMass,
  body-space invInertia), `PhysicsMaterial` (restitution, friction, compliance), inertia
  helpers (`solidSphere{Inertia,InvInertia}`, `worldInvInertia`).
- **Integration** (pure kernels): semi-implicit `integrateLinear`; orientation via the SO(3)
  **exponential map** (`so3ExpMap`/`so3LogMap`/`integrateOrientation`) — differentiable-ready
  (§14 constraints), not add-and-renormalize.
- `physics::Real` localizes the scalar type for a future double/dual-number switch.
- Driver: `tst/physics_test` (analytic checks: free-fall closed form, contacts, exp/log
  orientation, inertia). **Not yet**: `PhysicsWorld` interface + realtime backend (Phase 1),
  GJK/EPA + broadphase (Phase 2), implicit/differentiable backend + parallel worlds (Phase 3).

## ECS module (`engine::ecs`, 2026-07-03)

Archetype (table) ECS, std-only + `engine::core` (no graphics/physics dependency — they're
consumers). Design: investigations/2026-07-03-ecs-plan.md.

- `Entity` = `core::Handle<EntityTag>` (generational). `World` maps entity index → generation
  + location `{archetype, row}`.
- **Archetype storage**: entities sharing a component set live in one table with contiguous
  per-component columns (SoA); `signature` = sorted component ids. Swap-remove on destroy.
- **Components**: any trivially-copyable struct; compile-time `componentId<T>()` (static
  counter). The first shipped common component is **`engine::Transform`** (in `core`).
- `World::spawn<Ts...>` / `get<T>` / `has<T>` / `destroy`; `query<Ts...>().each(fn)` (per
  entity) / `.chunks(fn)` (per-archetype contiguous spans). Archetype iteration order is
  stable (creation order) → deterministic.
- **Resources**: typed singletons on the World (`setResource<T>`/`getResource<T>`), e.g.
  `Time{dt}`.
- **Scheduler** (`Schedule`): an ordered list of `void(World&)` systems, run in insertion
  order (deterministic). Systems read resources + iterate via queries. Driver
  `tst/scheduler_test` (gravity→integrate, hand-verified deterministic result).
- Not yet: command buffer for deferred structural changes, add/remove-component, parallel
  worlds / within-system parallelism (all planned).
- Driver: `tst/ecs_test` (spawn 1500 across 2 archetypes, query/mutate/chunk/destroy, verified).

`core` additions: `core/memory/handle.h` (`Handle<Tag>`, shared by rhi + ecs — the rhi's
`Handle` is now an alias) and `core/math/transform.h` (`engine::Transform`).

## Scene module (`engine::scene`, 2026-07-03)

The ECS↔render bridge — the only module that depends on **both** `engine::ecs` and
`engine::graphics` (keeps `ecs` graphics-free and the `Renderer` ECS-free).
- **Render components**: `RenderMesh { render::MeshHandle }`, `RenderMaterial { uint32_t
  materialIndex }` (trivially copyable → usable as ECS components).
- **Extraction system**: `scene::extract(World, pipeline, ExtractedScene&)` queries
  `<Transform, RenderMesh, RenderMaterial>`, buckets instances by mesh (one instanced draw per
  mesh), and fills `RenderItem[] + InstanceData[]` for a `render::RenderView`.
- Driver: `tst/scene_offscreen` (ECS entities → extract → render → pixel-verified);
  `tst/visual_window` is now ECS-driven (a bob system + extraction each frame).

## What is NOT here yet

- ECS: **archetype core exists** (`engine::ecs`: Entity/World/archetype storage/queries) —
  see the ECS module section above. Still missing: command buffer + add/remove-component,
  system scheduler + resources, render-extraction system, parallel worlds.
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
