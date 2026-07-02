# TODO

Gaps between [goals](goals.md) and [current state](architecture.md), roughly prioritized.
Not commitments — a backlog to reason about.

## Foundational (blocks most other work)

- [ ] **ECS core.** Decide on approach (archetype vs sparse-set) and stand up
      entity/component storage + system scheduling. This is the stated organizing model
      and nothing depends on it yet because it doesn't exist.
- [ ] **Driver test harness (not a `main`).** The engine is a library with no application
      entry point; consuming apps own the loop. Instead build a suite of driver tests under
      `tst/` that construct subsystems and drive the update→render loop to validate them.
- [ ] **Break the window/swapchain coupling.** Introduce a headless device/context path so
      ML training and offline rendering can run without GLFW. Today `Graphics` assumes a
      window + swapchain end to end.

## Driving milestone: "a ball rolling down a plane"

See goals.md. Long-term vertical slice: sphere rolling down an inclined plane, plus the
capability targets that make it exercise our goals — headless + deferred rendering, parallel
sims for training, and massive scale (~100k spheres). This is the yardstick for whether core
is "good enough" and defines what the graphics refactor pass must support.

## Core (in progress — build standalone first)

Build `engine::core` to a good state before touching graphics. See
[2026-07-02-core-module-plan.md](../investigations/2026-07-02-core-module-plan.md).

- [ ] **geometry/**: `Vertex` (position, **normal**, uv, color), `MeshData`, `Mesh`,
      minimal `Material`, `ModelData`. + `core.h` umbrella.
- [ ] **geometry/primitives**: `makeQuad` / `makePlane` / `makeSphere` generators (feed the
      milestone + give `tst` real geometry without asset files).
- [ ] **image/**: `Image` + `ImageFormat`.
- [ ] **io/**: `readFile`, `loadObj` → `ModelData`, `loadImage` → `Image`. NOTE: centralize
      the stb/tinyobj implementation macros in exactly one TU (they currently live in
      graphics) to avoid duplicate symbols when both link — do this when graphics is
      refactored, or keep loaders' impls out of any target that also links graphics.
- [ ] **tst driver**: build meshes via primitives, assert counts. Link `engine::core` only.

Scaling / ownership (from [2026-07-02-geometry-scaling.md](../investigations/2026-07-02-geometry-scaling.md)):
- [ ] Treat `MeshData`/`ModelData` as **loader output only** — not the runtime store; never
      hold ~100k of them (scattered heap allocs + deep-copy value semantics don't scale).
- [ ] **Central geometry store (pooled vertex/index arenas) + generational `MeshHandle`s** as
      the scalable runtime store; entities reference geometry by handle, not pointer. This is
      the trigger to un-defer `memory/` (`Handle`/`slot_map`) — build the minimal version
      inside this store first, promote to core only if a 2nd consumer needs it.
- [ ] Drive the milestone's 100k spheres via **instancing** (one geometry + 100k transforms),
      with per-frame data in **flat SoA** render/instance arrays (ties to the DrawJob rework).
- [ ] Derive **bounds / position-only data** for physics/culling instead of walking
      interleaved `Vertex` arrays on CPU hot paths.
- [ ] Later/at-scale: **vertex compression** (compact GPU layout, keep core::Vertex as the
      authoring format) and **distinct-geometry streaming/LOD**; decide CPU-retention policy
      (don't keep CPU vertex copies after upload except for collision/raycast).

## Graphics refactor pass (LATER — after core is solid)

**Do NOT refactor graphics incrementally.** Once core is in a good state, refactor the entire
graphics package in one dedicated pass, behind the RHI. Full analysis:
[2026-07-01-graphics-refactor.md](../investigations/2026-07-01-graphics-refactor.md) and
[2026-07-02-metal-backend.md](../investigations/2026-07-02-metal-backend.md).
**Key framing: "add Metal" and "do the refactor" are the same effort** — both require
extracting a backend-agnostic interface (RHI) and putting Vulkan behind it.

- [x] **0. Decide: build-your-own RHI vs. adopt one.** DECIDED (2026-07-02): **build our
      own** — no third-party RHI. Both Vulkan and Metal backends are hand-written.
- [ ] **1. Adopt `engine::core` in graphics**: replace graphics' `Vertex`/`Mesh`/`Material`
      and its OBJ/image loading with core's; delete the now-duplicated code. Each backend
      maps `core::Vertex` to its own vertex format. (Core itself is built in the Core section
      above, before this pass.)
- [ ] **2. Define the RHI interface** (Device/Swapchain/Buffer/Texture/Pipeline/CommandBuffer
      /Fence) with **no `Vk*`/`MTL::` in the public surface**. This is the refactor's Device
      split + first-class pipelines, expressed as the cross-backend contract.
- [ ] **3. Reorganize existing Vulkan code** into `src/graphics/vulkan/` behind the RHI;
      delete the 3 duplicated pipeline builders; fold `graphics_custom.cpp` offscreen helpers
      into the pipeline/pass API. (Design Vulkan side around dynamic rendering to line up
      with Metal.)
- [ ] **4. Fix multi-backend CMake** (details in metal-backend §6): enable `OBJCXX` on Apple,
      link **Foundation** framework, add `external/metal-cpp` to include dirs, one TU defines
      metal-cpp impl macros, **per-backend source selection** (not one recursive glob),
      rename the `test` target.
- [ ] **5. Shader toolchain** (now two targets): pick Slang (one source → SPIR-V + MSL) or
      GLSL+SPIRV-Cross; add a build step; abstract `rhi::ShaderModule` over `.spv`/`.metallib`.
- [ ] **6. Implement the Metal backend** incrementally: Device → CAMetalLayer/drawable (+ `.mm`
      window shim via `glfwGetCocoaWindow`) → Buffer/Texture → Pipeline → command encoding →
      first triangle → parity with Vulkan. Wrap metal-cpp objects in RAII; AutoreleasePool
      per frame.
- [ ] **7. Split `Swapchain` + `Renderer`; headless path** for both backends (ML/offline).
- [ ] **8. Rework the render-list / instance path** against the ECS interface once ECS
      exists (fatten `InstanceSSBO`, upload only what's used, consider indirect/bindless).

Known bugs/smells to fix along the way (details in the refactor investigation):
- [ ] `loadQuad` mis-computes `vertexCount`/`indexCount` (cumulative, not per-mesh counts).
- [ ] `copyInstanceToBuffer` always copies `MAX_ENTITIES`; dirty-range copy is stubbed out.
- [ ] `renderFinishedSemaphores` indexed per-frame not per-image (masked by frames=1).
- [ ] macOS `vkQueueWaitIdle` after every present + `MAX_FRAMES_IN_FLIGHT = 1` → no
      pipelining. Revisit with multiple frames in flight.
- [ ] "paint"/"stamp" naming leftovers from the source app.
- [ ] `GlobalUBO` lights (`// todo: lights`).
- [ ] Deferred deletion queue (readme.md sketch), once resources outlive a frame.

## Compute / ML

- [ ] **Compute pipeline + compute-queue support.** Needed for ML workloads and some
      rendering; only graphics pipelines exist today.
- [ ] **Parallel environment stepping / batching** for training throughput.
- [ ] **Determinism review** for reproducible training.

## Physics

- [ ] Replace the stub with a real simulation core (integrator, collision, broadphase).
- [ ] Decide how physics state maps onto ECS components.

## Infra / quality

- [ ] Multithreaded task system (readme.md sketches work-stealing + task dependency graph).
- [ ] Test setup (no framework wired up yet).

## Open design questions (revisit "at some point")

Parked decisions we agreed to defer. Answer before the work they gate.

- [ ] **ECS approach: archetype vs sparse-set.** Archetype gives great cache/iteration
      behavior and pairs well with the batched render path; sparse-set is simpler and more
      flexible for add/remove. Gates the ECS core.
- [ ] **ML training model: many envs in-process vs separate processes.** Drives the
      headless context design and how far determinism must reach.
- [ ] **Offline renderer basis:** is `graphics_custom.cpp`'s offscreen helper set intended
      to grow into the offline/high-quality renderer?
- [x] **Build-your-own RHI vs. adopt.** DECIDED (2026-07-02): build our own; no third-party
      RHI. (metal-backend §3)
- [ ] **Shader language/toolchain**: Slang (one source → SPIR-V + MSL) vs. GLSL + SPIRV-Cross.
- [ ] **RHI dispatch**: runtime-virtual (simpler) vs. compile-time (zero overhead).
- [ ] **Metal API level**: classic `MTL` (widest support) vs. `MTL4` (newer, Vulkan-like;
      present in vendored metal-cpp).
- [ ] **Future D3D12 / Windows?** If likely, argues harder for adopting an RHI now.
