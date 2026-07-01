# TODO

Gaps between [goals](goals.md) and [current state](architecture.md), roughly prioritized.
Not commitments — a backlog to reason about.

## Foundational (blocks most other work)

- [ ] **ECS core.** Decide on approach (archetype vs sparse-set) and stand up
      entity/component storage + system scheduling. This is the stated organizing model
      and nothing depends on it yet because it doesn't exist.
- [ ] **Application/loop layer.** There's no `main` or driver. Need an entry point that
      owns the engine lifecycle and the update→render loop.
- [ ] **Break the window/swapchain coupling.** Introduce a headless device/context path so
      ML training and offline rendering can run without GLFW. Today `Graphics` assumes a
      window + swapchain end to end.

## Rendering

Graphics refactor — see [investigations/2026-07-01-graphics-refactor.md](../investigations/2026-07-01-graphics-refactor.md)
for full analysis. Sequenced so the build stays green at each step:

- [ ] **1. Extract `engine::core`** (backend-agnostic): move `Vertex` (as pure data),
      `Mesh`, `Material`, and OBJ loading out of graphics; add a Vulkan `vertex_format`
      mapping in graphics. Unblocks physics sharing geometry.
- [ ] **2. Split out `Device`/`VulkanContext`**: instance/device/queues/command pool + RAII
      resource types (Buffer/Image/Texture). Consider VulkanMemoryAllocator (VMA).
- [ ] **3. First-class pipelines**: `PipelineConfig` + `PipelineBuilder` with defaults;
      delete the 3 duplicated ~150-line pipeline builders. Add RenderPass/DescriptorSet
      layout builders. Fold the `graphics_custom.cpp` offscreen helpers into this API.
- [ ] **4. Split `Swapchain` + `Renderer`**; introduce a headless (no-window) path built on
      the existing `setInstance`/`setSurface` seams.
- [ ] **5. Rework the render-list / instance path** against the ECS interface once ECS
      exists (fatten `InstanceSSBO`, upload only what's used, consider indirect/bindless).

Known bugs/smells to fix along the way (details in the investigation):
- [ ] `loadQuad` mis-computes `vertexCount`/`indexCount` (cumulative, not per-mesh counts).
- [ ] `copyInstanceToBuffer` always copies `MAX_ENTITIES`; dirty-range copy is stubbed out.
- [ ] `renderFinishedSemaphores` indexed per-frame not per-image (masked by frames=1).
- [ ] macOS `vkQueueWaitIdle` after every present + `MAX_FRAMES_IN_FLIGHT = 1` → no
      pipelining. Revisit with multiple frames in flight.
- [ ] "paint"/"stamp" naming leftovers from the source app.
- [ ] Shaders in-tree + SPIR-V build step (pipelines currently take runtime shader paths).
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
