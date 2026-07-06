# Renderer head-swap readiness — module dependency review (2026-07-06)

Groundwork for the (planned) path tracer: can we swap the **graphics head** between the realtime
renderer and a path tracer without disturbing physics or the shared core? Review of the current
module boundaries, verified against CMake link deps + every cross-module `#include` (not assumed).
Analysis only — nothing changed here; recommendations feed `core/todo.md` when the path-tracer track
starts.

## Verified dependency graph

```
core            → (external only: glm, tinyobjloader, stb)   ← pure foundation, no engine deps
ecs             → core
physics         → core
graphics        → core (+ glfw, metal|vulkan backend)
input           → core            input_glfw → input
physics_ecs     → physics + ecs        (physics↔ECS bridge)
physics_env     → physics + core       (RL env bridge)
scene           → ecs + graphics       (ECS→render bridge)
controls        → ecs + input
```

## physics ⊥ graphics — TRUE, and enforced

Confirmed at two levels:
- **Link level:** `engine::physics → engine::core` only; `engine::graphics → engine::core` only.
  Neither target references the other.
- **Include level:** grepped every `.h/.cpp/.mm` in both trees — **zero** `engine/graphics/*` includes
  in physics, **zero** `engine/physics/*` includes in graphics.

The only files pulling in both are `tst/physics/visual/*` (rolling, humanoid, reduced_joints,
diff_humanoid, amp_humanoid) — demo **apps** (the composition root), the correct place for the two to
meet. There is **no module that bridges physics *and* graphics**; they are siblings over `core`.

**How they meet without knowing each other:** physics writes `Transform`s into the ECS →
`scene::extract` queries `<Transform, RenderMesh, RenderMaterial>` and emits a `RenderView` → the
Renderer consumes it. `scene` depends on graphics but not physics; `physics_ecs`/`physics_env` depend
on physics but not graphics. Clean data-flow decoupling — no cleanup debt on this axis.

## The head-swap seam: `render_view.h`

`render_view.h` is the intended boundary and says so ("plain data… No Vk\*/MTL:: and no ECS types
here"). It's the right seam, but **not fully head-neutral yet** — a few realtime-raster concepts leak
into what should be a "what's in the scene" contract:

1. **`RenderItem::pipeline` (raster `PipelineHandle`)** + batching semantics ("one batchable instanced
   draw") are raster-specific. A path tracer binds no per-item raster pipeline; it wants geometry + an
   acceleration structure + a global RT/compute pipeline.
2. **`scene::extract(world, pipeline, out)` takes a pipeline handle** and stamps it on every item — so
   ECS→scene extraction is coupled to a realtime concept. A path tracer wants the same extraction to
   produce pipeline-free scene data.
3. **`GeometryStore` / `MeshHandle`** are raster vertex/index arenas. A path tracer shares the *source*
   geometry (`core::geometry`, already a neutral CPU layer) but needs its own GPU residency
   (BVH/accel), not raster vertex/index buffers.
4. **`GraphicsConfig` / `RenderResources`** are entirely realtime (MSAA/FXAA/tonemap/shadow/froxel). A
   path tracer gets its own config (spp, max bounces, denoiser). Not shared — just noting.
5. Already head-**neutral**: `DirectionalLight`, `PointLight`, `MaterialGPU`, camera matrices,
   `target` (RenderTargetHandle), `clearColor`.

## Structural observation: `graphics` fuses two layers

- `graphics/rhi/` — backend abstraction (Device/CommandList/resources/pipeline). **Head-neutral**;
  both renderers build on it (the path tracer via compute).
- `graphics/render/` — the realtime renderer specifically (renderer, render_graph, frame_ring,
  geometry_store, graphics_config).

They're a single CMake target (`engine::graphics`), and `RenderView` — the shared contract — lives
*inside* `render/` next to realtime-only code.

## Recommendations (do when the path tracer forces it, not before)

- **Promote the neutral contract out of `render/`:** move `render_view.h` (and likely `MeshHandle`)
  to a head-neutral spot (`graphics/rhi/` or a small `graphics/scene_view/`), drop the raster
  `pipeline` off `RenderItem` (let each head decide how to draw), and make `scene::extract`
  pipeline-free.
- **Split the target when the second head lands:** `engine::rhi` (neutral base) → `engine::render_realtime`
  + `engine::render_pathtrace`, with `scene` depending only on the neutral contract. Don't pre-split
  now — cheap to do once there's a second consumer.
- **Backend-swap confidence (separate from head-swap):** the Vulkan backend is stale (pre-RHI-refactor
  god-object, `src/graphics/vulkan/graphics.cpp` ~48 KB from Jun 12). The RHI's backend-agnosticism is
  currently proven only by Metal; bringing Vulkan onto the new RHI would validate that boundary.

## Bottom line

The physics/graphics separation is solid and enforced — no work needed there. The head-swap work is
concentrated in **de-rasterizing the `RenderView` contract** and (later) **separating `rhi` from
`render/`**. Both are small and best done as the path tracer introduces the second consumer.
