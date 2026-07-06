# Head-swap refactor — execution plan + locked decisions (2026-07-06)

Follows the readiness review ([2026-07-06-renderer-head-swap-readiness.md](2026-07-06-renderer-head-swap-readiness.md)).
The prep work to make the graphics head swappable (realtime | path-tracer) *before* the path tracer
is written. Behavior-preserving throughout; the full suite (167/0) must stay green after each phase.

## Locked decisions
1. **Pipeline removal = option A (renderer-held mesh pipeline).** Drop `RenderItem::pipeline`; the
   realtime renderer holds a single opaque mesh pipeline (`RenderResources.mesh` + `setMeshPipeline()`),
   bound once per forward pass. Matches today's single-pipeline-per-view reality → behavior-preserving.
   Multi-pipeline scenes (transparency, per-material shaders) become a future realtime-**internal**
   feature (material→pipeline map), NOT a contract change (option B, deferred).
2. **Keep the `RenderView` / `RenderItem` names.** A rename is pure churn across ~25 files for a
   cosmetic gain; the header already documents the neutral intent.
3. **Do the module split now** (not deferred to path-tracer start): lock the boundary in the build
   graph while realtime is the only consumer (lower-risk than splitting mid-path-tracer).
4. **Test layout:** split `tst/graphics` into `tst/graphics/realtime/` + `tst/graphics/path-tracer/`
   (mirrors the notes reorg). Physics + other tests stay as-is and use the realtime renderer.

## Phases (each independently builds + passes 167/0)

**P1 — De-rasterize the contract.**
- `render_view.h`: `RenderItem { mesh, firstInstance, instanceCount }` (drop `pipeline`).
- Realtime renderer: add `RenderResources.mesh` + `setMeshPipeline()`; forward pass binds it once
  before the item loop (was `bindPipeline(item.pipeline)` per item).
- `scene::extract(world, out)` loses its `pipeline` param.
- Update all ~20 `RenderItem` construction sites + the 8 `extract()` callers (set the mesh pipeline
  on the renderer). Behavior-preserving (all current scenes are single-pipeline-per-view).

**P2 — Relocate contract + split the target.**
- Move `render_view.h` (+ `MeshHandle`) to `include/engine/graphics/view/`; fix direct includers.
- Split `engine::graphics` (keep the `include/engine/graphics/*` tree) into:
  - **`engine::rhi`** — `rhi/` + `view/` + backend (`src/graphics/{metal,vulkan}`, `metal_window.mm`).
    The neutral GPU foundation + scene contract. Both heads build on it.
  - **`engine::render`** — `render/` + `src/graphics/common/{renderer,geometry_store}`, the realtime
    renderer; `PUBLIC engine::rhi`.
- Rewire deps: `scene` → `engine::rhi` only (it needs just the neutral contract, tightening its
  current over-link to all of graphics); tests/benchmarks/visuals link `engine::render`.
- Future path tracer = `engine::pathtracer` over `engine::rhi` (a sibling of `engine::render`).
- `GeometryStore` (raster vertex/index residency) stays in `engine::render`; the path tracer builds
  its own accel-structure geometry from the shared `core::geometry` source.

**P3 — Reorganize graphics tests.**
- `git mv tst/graphics/{unit,integration,benchmark,visual}` → `tst/graphics/realtime/{...}`.
- Create `tst/graphics/path-tracer/` (placeholder, ready for the first PT test).
- Update `tst/CMakeLists.txt` globs (`graphics/realtime/**`, `graphics/path-tracer/**`). `TST_CASE`
  module stays `graphics` (sub-organized by head). Physics/other test trees unchanged.

## Notes
- RHI-level tests (`barrier_transient`, `compute_smoke`, `rhi_smoke`) move under `realtime/` for now
  even though they exercise the neutral base — revisit hoisting them to a `graphics/rhi/` tree if the
  path tracer wants to share them.
- No behavior change anywhere; this is a structural + contract refactor only.
