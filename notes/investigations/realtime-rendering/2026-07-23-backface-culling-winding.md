# Back-face culling is not applied — winding convention needs an audit (2026-07-23)

Point-in-time finding, discovered while adding the per-item pipeline / foliage support in the
outdoor-arena milestone ([2026-07-23-outdoor-arena-milestone.md](2026-07-23-outdoor-arena-milestone.md)).
Deferred fix; this note is the record so the next engineer doesn't rediscover it.

## The finding

`rhi::GraphicsPipelineDesc.raster` carries a `CullMode` (default **`Back`**) and a `FrontFace`
(default **`CounterClockwise`**), but **the Metal backend never applies them.** `createGraphicsPipeline`
stored nothing from `desc.raster`, and `CommandList::bindPipeline` never called `setCullMode` /
`setFrontFacingWinding` — so every draw runs with Metal's default `MTLCullModeNone`. **Nothing is
culled today.** The API advertises culling it doesn't do.

## Why we can't just turn it on

Enabling it naively (store `cull`/`winding` in the pipeline slot; apply on the encoder at bind) was
tried and **reverted** — it broke **9 tests**:

- `graphics.mesh`, `graphics.material_features`, `graphics.multi_light`, `graphics.multiview_ring`,
  `graphics.scene` — all render spheres and went (mostly) to the clear color: `cull=Back` culled the
  **front** faces.
- `graphics.shadow_map`, `graphics.shadow_bias` — shadow/caster geometry dropped out.
- the two new foliage tests.

Root cause: **the engine's geometry is wound assuming no culling.** With the default
`FrontFace=CounterClockwise`, the *outward* faces of `primitives::make*` meshes are **clockwise** in
screen space ⇒ classified back-facing ⇒ removed by `cull=Back`. The whole content pipeline (and every
pixel-parity test) was authored against "no culling," so flipping the switch inverts what's visible.

Contributing factors to double-check during the fix:
- `GLM_FORCE_DEPTH_ZERO_TO_ONE` + the projection's Y handling affect *screen-space* winding vs the
  authored CPU winding.
- Fullscreen passes (`sky`, `tonemap`, `fxaa`) draw a single triangle and must be `CullMode::None`
  (or wound to survive whatever convention we pick).
- The depth-only **shadow pass** pipeline needs the same treatment as the forward mesh pipeline.

## What a proper fix requires (do this when we revisit)

1. **Pick and document a front-face convention** (recommend `CounterClockwise` front, `Back` cull —
   the GL/Vulkan-common default) in `goals.md`/`architecture.md`.
2. **Make every geometry source consistent** with it:
   - `src/core/geometry/primitives.cpp` (`makeQuad/makePlane/makeSphere/makeBox/makeCapsule`) — fix
     triangle index order so outward faces are front.
   - `src/core/geometry/obj_loader.cpp` — confirm tinyobj triangulation winding; flip if needed.
   - any other mesh producers (ECS/scene, path-tracer shares the catalog but doesn't raster-cull).
3. **Apply `cull`/`winding` in the backend** (store in the pipeline slot; `setCullMode` +
   `setFrontFacingWinding` at bind — the reverted change is the starting point).
4. **Set fullscreen/sky pipelines to `CullMode::None`** explicitly.
5. **Re-verify pixel parity** across all `graphics.*` tests (they are the winding oracle) and add a
   dedicated cull test (front-facing quad visible, back-facing quad culled).
6. Only then does `CullMode::None` on a foliage pipeline actually mean "draw both sides."

## Root cause — IDENTIFIED (review, 2026-07-23)

Reviewed `primitives.cpp`, `camera.h`, and the Metal viewport/projection path. The precise mechanism:

- **All `primitives::make*` meshes are wound counter-clockwise around their OUTWARD normal** (the
  right-handed / glm / OpenGL convention). Verified: `makePlane` is commented "CCW when viewed from
  +Y"; and `cross(edge1, edge2)` for each primitive's first triangle equals the vertex outward
  normal (checked quad/plane/box). So the CPU geometry is **consistent and CCW-front** — nothing is
  mis-wound.
- **No Y-flip exists anywhere**: the projection is `glm::perspective` used directly (RH, 0..1 z), and
  the Metal viewport uses a positive height (`MTL::Viewport{x,y,w,h,...}`). Images are upright because
  Metal NDC is Y-up and the viewport maps NDC +Y to the top of the (top-left-origin) framebuffer.
- **Metal decides front/back facing in framebuffer (window) space**, where that NDC→framebuffer map
  NEGATES Y. A single-axis negation flips triangle orientation, so a **world/NDC CCW-outward triangle
  is CLOCKWISE in framebuffer space.**
- So the correct Metal front-facing winding for this engine's geometry is **`MTL::WindingClockwise`.**
  The reverted attempt mapped the RHI default `FrontFace::CounterClockwise` → `MTL::WindingCounterClockwise`,
  telling Metal "CCW-window = front" — but the outward faces are CW-window ⇒ classified **back** ⇒
  `cull=Back` deleted exactly the visible faces. That's the whole bug.

### Corrected fix (smaller than first thought — NO primitive rewind, NO projection change)

The CPU winding is already correct and consistent; the fix is purely the backend's winding
interpretation + a couple of pipeline settings:

1. **In the Metal backend, account for the framebuffer Y-flip**: map the RHI's *logical world-space*
   `FrontFace::CounterClockwise` → `MTL::WindingClockwise` (and `Clockwise` → `CounterClockwise`).
   Keep the RHI convention "front = CCW around the outward normal in world space" (matches the
   primitives + glm + a future Vulkan backend); the Y-flip inversion lives in the backend.
2. **Set fullscreen/sky/tonemap/fxaa pipelines to `CullMode::None`** (they draw a single triangle with
   no meaningful winding).
3. The depth-only **shadow pass** renders the same meshes — give it the same winding mapping; culling
   front vs back for shadow maps is a later bias/peter-panning tuning, not correctness.
4. **Re-verify all `graphics.*` pixel parity** (they are the winding oracle) + add a cull test
   (front-facing quad visible, back-facing quad culled).

So the earlier "audit/rewind every geometry source" worry (step 2 of the original checklist above) is
**not needed** — geometry is already uniform. The real work is the backend CCW→Clockwise mapping plus
marking the fullscreen pipelines `CullMode::None`.

## RESOLVED (2026-07-23)

Fixed. Back-face culling is now applied and correct. What the fix actually took (the review above
under-counted it — the primitives were not merely "CCW-outward but Y-flipped," they were **internally
inconsistent**):

- **`makeSphere` / `makeCapsule` were uniformly CW-outward** — the `(φ,θ)` parameterization has
  `∂φ×∂θ = −sinφ·n` (inward). Flipped their triangle template `{i0,i2,i1,…}` → `{i0,i1,i2,…}` so they
  wind CCW around the outward normal.
- **`makeBox` was per-face inconsistent** — its `+Y`/`−Y` faces had `(u,v,n)` left-handed
  (`cross(u,v) = −n`), so those two faces were CW-outward while the other four were CCW-outward.
  (This is why the shadow caster's top face was being culled → "no shadow patch.") Reordered their
  `(u,v)` so `cross(u,v)=n` for all six faces.
- `makeQuad` / `makePlane` were already CCW-outward — unchanged.
- **Backend now applies `RasterState.cull` + `frontFace`** (`setCullMode` + `setFrontFacingWinding` at
  bind). With every primitive uniformly CCW-outward, the winding maps **1:1** — RHI
  `FrontFace::CounterClockwise` → `MTL::WindingCounterClockwise` (NO Y-flip inversion; the earlier
  theory about a framebuffer Y-flip was wrong — glm's RH projection + Metal preserve CCW-front).
- The fullscreen passes (sky/tonemap/fxaa, single triangle) turned out to be wound front-facing, so
  they survive `cull=Back` and did **not** need `CullMode::None`. (hdr_tonemap already sets None.)

**Convention (now enforced):** front faces are wound **counter-clockwise around the outward normal in
world space** (glm/OpenGL convention). All `primitives::make*` and `loadObj` output follow it; the
`Renderer::createMeshPipeline` factory defaults to `CullMode::Back`.

Verified: full graphics pixel parity held (spheres/boxes/planes/shadows unchanged), and a dedicated
`graphics.backface_cull` test (front-facing drawn 765, back-facing culled ≈ clear 82, `cull=None`
draws both). `CullMode::None` on a foliage pipeline now genuinely means "draw both sides." Suite 195/0.

## Original interim state (as of 2026-07-23, before the fix)

- Cull is **not applied**; `MeshPipelineVariant.cull` / `RasterState.cull` are accepted but a **no-op
  for culling** in the Metal backend.
- **Two-sided *lighting*** (`MaterialFlagDoubleSided`, normal flip on back faces via `SV_IsFrontFace`)
  is independent of culling and **works** — so foliage is correctly lit on both sides even though
  nothing is culled.
- Consequence: no back-face-cull perf win yet, and a single-sided cutout card is visible from both
  sides regardless of its material flags. Acceptable for now; revisit per the todo item
  "honor `RasterState.cull` in the backends."
