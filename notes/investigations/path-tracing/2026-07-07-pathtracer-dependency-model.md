# Path-tracer dependency model + scene bridge — review (2026-07-07)

Follows step 1 (the CPU reference tracer). Before wiring the path tracer into engine scenes, settle
the dependency model so the offline head stays cleanly decoupled — specifically: **`engine::pathtracer`
must depend on `core` only (never `ecs`, `rhi`, or `render`)**, mirroring `engine::physics`.

## Locked decisions (owner, 2026-07-07)
1. **Materials: keep the single shared model for now.** Revisit later by subclassing into a realtime
   material + a path-tracing material once they genuinely diverge.
2. **CPU geometry copy is fine** — but it must not drag the path tracer into a bad dependency (see below).
3. **Lights come from emissive materials** (area lights via emission). No analytic directional/point
   lights needed for the path tracer.
4. **No runtime A/B** required — two static images side by side (realtime | path-traced) is acceptable.

## Current facts (from the code)
- `engine::pathtracer` → `core` only. ✓ `pt::Scene::addMesh` already consumes `core::MeshData`.
- **Geometry identity is currently a *render* concept.** `render::MeshHandle` (in `rhi/view/`) indexes
  `GeometryStore` (in **`engine::render`**), which owns the GPU arenas *and* keeps a private CPU copy of
  the vertices/indices. `core::MeshData` is explicitly transient ("loader output; the authoritative
  store is the pooled buffers + MeshHandles" — geometry-scaling note). There is **no CPU geometry
  residency independent of the realtime renderer.**
- ECS `scene::RenderMesh { render::MeshHandle }` ties the ECS scene to `GeometryStore` handles — i.e.
  to the realtime head.

## The coupling risk
A naive ECS→`pt::Scene` adapter that resolves geometry by looking up `RenderMesh`'s `MeshHandle` in the
`GeometryStore` would make the **path-trace path depend on `engine::render`** — the offline head reaching
through the realtime head for triangles. That's the confusion to avoid.

## Key refinement: `RenderView` is the *raster head's* contract, not a head-neutral scene
The head-swap review framed `RenderView` as the neutral contract both heads consume. Building the PT head
shows that was optimistic: `RenderView` carries **GPU mesh handles + draw batching + no emission** — it's
the raster head's input. The path tracer wants **CPU triangles + emission + (later) a BVH**. Those are
genuinely different representations.

So the truly-neutral layer is **the scene as core data** (geometry + identity, transforms, materials with
emission, camera). Each head has a **bridge** projecting that into its own representation:

```
              core scene data (geometry catalog + transforms + materials + camera + emissive lights)
                 │                                                    │
  engine::scene (ecs+rhi) ── RenderView ─► engine::render   engine::pathtracer_scene (ecs+pathtracer) ── pt::Scene ─► engine::pathtracer
     (raster bridge; EXISTS)                (raster head)        (PT bridge; NEW)                                      (PT head; core-only)
```

**Why `pathtracer` needs a `pathtracer_scene` but `render` seemingly doesn't:** `render` *does* have a
bridge — it's `engine::scene` (ECS→`RenderView`), just not named `render_scene` (historical: it was the
only render bridge). The PT head needs its own bridge because it consumes a *different* contract
(`pt::Scene`, not `RenderView`); folding both into `engine::scene` would couple the two heads' bridges and
make `scene` depend on both `rhi` and `pathtracer`. Both bridges are optional — an app/test can build
either contract by hand (the Cornell + headless render tests do); the bridge only serves the ECS-driven case.
Analogy: `pathtracer : pathtracer_scene :: physics : physics_ecs`. (Could rename `scene`→`render_scene` for
symmetry someday; not necessary.)

## Enabling refactor: hoist geometry identity + CPU residency to `core`
For `pathtracer_scene` to build a `pt::Scene` **without** depending on `engine::render`:
- Add a **core geometry catalog** owning `MeshData` keyed by a `core::MeshId` (CPU-side, core-only).
- `GeometryStore` (render) uploads *from* the catalog, mapping `MeshId → GPU DrawRange`.
- `pathtracer_scene` reads `MeshData` *from* the catalog via `MeshId`.
- ECS `RenderMesh` references a **`core::MeshId`** (not `render::MeshHandle`) — decoupling the ECS scene
  from any renderer. `scene::extract` maps `MeshId→GPU`; `pathtracer_scene` maps `MeshId→MeshData`.
- Bonus: the BVH builder (step 3) wants exactly this core CPU geometry — the catalog pays for itself twice.

Resulting graph:
```
core                       ← geometry catalog + MeshData + MeshId
ecs              → core
pathtracer       → core                        (offline head; core-only ✓)
rhi              → core
render           → rhi, core                   (raster head)
scene            → ecs, rhi, core              (raster bridge → RenderView)
pathtracer_scene → ecs, pathtracer, core       (PT bridge → pt::Scene)   ← NEW
```

## Near-term: the side-by-side demo needs none of this yet
Two static images (raster | path-traced) don't require the bridge or the catalog: a demo already holds the
source `core::MeshData` + transforms + materials, so it can feed the raster path (upload→GeometryStore→
texture A) *and* the PT path (`pt::Scene::addMesh` from the same MeshData → CPU render → texture B)
directly. `pathtracer` stays core-only; no new module. The bridge + catalog matter only when the
**authoritative scene lives in the ECS** and we want to path-trace *that*.

## Recommendation + open decision (sequencing)
- Keep `pathtracer` core-only (done); commit to the `pathtracer_scene` bridge pattern for the ECS path.
- Do the core geometry-catalog refactor as the enabling step (also unblocks the BVH).
- **Open: order.**
  - **(A)** core catalog + `pathtracer_scene` bridge first, then the side-by-side demo renders the ECS
    scene through both heads (proves the real pipeline; more upfront refactor).
  - **(B, leaning)** side-by-side demo now, feeding `pt::Scene` directly from core geometry (no bridge);
    land the catalog + bridge as a focused follow-up with a concrete consumer in hand.

## Also settled here (material extension for PT)
Emission + a BSDF `type` need to exist on the shared material for PT to have lights/specular. Per decision
1 that stays one model for now (extend the shared material minimally). `pt::Material` already has
`albedo/emission/type/ior/shininess`; the shared `render::MaterialGPU` (currently `baseColorFactor` +
texture) would gain `emission` (+ a type/flags field) when the bridge needs it.
