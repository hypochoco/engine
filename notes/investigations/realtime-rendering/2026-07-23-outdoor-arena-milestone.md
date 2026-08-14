# Milestone: a small, performant outdoor arena scene (2026-07-23)

Point-in-time plan. Folds into `core/todo.md` + `core/goals.md`; this doc holds the reasoning.

## The goal

Render a **small, good-looking outdoor arena** — the kind of bounded space you'd stage a fight
in. Concretely:

- A **grassy patch of ground** with **dirt patches** and scattered **small rocks**.
- **A little local atmosphere** around the space (haze/depth, sun in the air).
- **A tree**, with **leaves and grass waving in the wind**.
- **Performant** — this is an *arena for (later) fighting characters*, so the static scene must
  leave a large frame-time budget for animated, simulated characters on top.

Delivered as a **separate game repo** that takes this engine as a dependency and builds the
scene on top of it. This is the first real consumer of the engine as a library, and it directly
exercises the `goals.md` "engine ↔ application split": the engine provides *mechanisms*
(RHI, render graph, ECS, materials, culling), the game provides *content and shaders*
(the arena, its assets, its pipelines, the wind/ground-blend/grass shaders).

Non-goals for this milestone (tracked, deliberately later): the fighting characters themselves
(skinned meshes + animation + control), networking, and full non-flat terrain
(collision/heightfield). A flat grassy ground plane is enough for the arena.

## Why this is a good milestone

- It's the **first end-to-end game-on-engine**, forcing the library boundary to be real and clean.
- It pushes the renderer past "colored primitives" into **textured, authored content** — the
  single biggest missing piece for anything that looks like a game.
- It stays **small and bounded**, so we can chase quality + performance without an open-world
  scope explosion.
- The atmosphere/sky/shadow/AA framework we already built (RF6) was designed for exactly this
  kind of lit outdoor scene — this cashes it in.

---

## Current engine state (verified against code, 2026-07-23)

Read: `core/architecture.md` (graphics + scene sections), `rhi/{device,resources,types,command_list}.h`,
`view/render_view.h`, `render/renderer.h`, `src/shaders/mesh.slang`, `core/geometry/*`,
`src/graphics/metal/metal_backend.cpp`, `external/`.

### What we HAVE (relevant to the arena)

- **RHI + Metal backend**: texture create-with-data, **mip levels** (allocated, see gap on
  generation), samplers (Nearest/Linear, mip modes, **Repeat/Clamp/Mirror**, **anisotropy**),
  blend presets incl. **AlphaBlend + Additive**, **MSAA**, **compute pipelines**, **indirect
  draw**, and **explicit single-texture/sampler binding** (`TextureBinding`/`SamplerBinding`,
  already used for the shadow map + tonemap input).
- **Render graph (clustered forward+)**: instanced indexed draws bucketed per mesh+material,
  per-instance `InstanceData` + per-material `MaterialGPU`, `FrameRingAllocator`, resource
  barriers.
- **Lighting/atmosphere**: ambient + Lambert **directional (sun)** + **point lights** (froxel
  clustered), **one directional shadow map** (2048², 3×3 PCF, slope-scaled bias), **analytic
  procedural sky**, **aerial perspective + height fog** (in `mesh.slang`), **MSAA + FXAA**,
  **HDR + ACES tonemap**, all behind a central **`GraphicsConfig`** (opt-in, off-by-default,
  parity-anchored).
- **ECS → render bridge**: `scene::extract` (`<Transform, RenderMesh, RenderMaterial>` → instanced
  `RenderView`), `scene::extractViews` (camera entities → views), `Background`/`SceneLighting`.
- **Input + camera**: backend-agnostic `input` + GLFW adapter + `FlyController` + camera-as-entity.
- **Geometry**: procedural primitives (quad, subdivided plane, sphere, box, capsule) +
  `core::GeometryCatalog` (authoritative CPU mesh residency, `MeshId`).
- **Vendored, not yet wired**: `stb_image.h` (image load), `stb_perlin.h` (noise, useful for
  ground masks + wind), `tinyobjloader` (mesh load); the parked Vulkan `graphics_model.cpp` has a
  `loadObj` to port from.
- **Physics**: articulated humanoid on a flat plane collider; fine for a flat arena floor.

### What we're MISSING (the gaps this milestone must fill)

Ordered roughly by how much stands between us and the scene:

1. **Textured surfaces (albedo) end-to-end.** The biggest gap. `MaterialGPU.baseColorTexture`
   exists but is *unused* — the mesh shader only does `vertexColor × baseColorFactor`. Blocked on:
   - **Asset loading**: no `core::io` (`readFile`) and no `core::image` (`Image` + `loadImage`
     via stb_image); no `core::geometry` OBJ loader.
   - **RHI bindless texture table is declared on `Device` but NOT implemented** in the Metal
     backend (`registerBindlessTexture` has no body). Material texturing needs many textures
     addressable from one draw → bindless (Metal argument buffer / heap) **or** a bounded
     texture-array/atlas fallback. Decision below.
   - **No mip generation** (only `mipLevels` allocation) → minification shimmer + bandwidth on
     ground/foliage. Need a blit `generateMipmaps`.
2. **Normal mapping (and therefore tangents).** `core::Vertex` has **no tangent** (explicitly
   deferred in `vertex.h`). Ground/rock/bark detail wants normal maps ⇒ add a tangent attribute
   (or compute per-vertex at load) + extend the vertex layout + `mesh.slang`.
3. **Richer material model.** `core::Material` / `MaterialGPU` are albedo-only. Need (at least)
   albedo + normal + roughness/metallic (ORM) + emissive, and an **alpha-cutout flag** (for grass
   / leaf cards). Keep additive + opt-in so existing pixel-parity tests hold.
4. **Alpha-tested, double-sided foliage path.** Grass blades and leaves are cutout cards:
   need `discard`-based alpha test + `CullMode::None`. Pipeline state already supports it; it's a
   material-flag + app-shader concern, plus verifying the render graph handles it.
5. **Wind vertex animation.** No vertex animation today. Grass bends from its base; leaves
   flutter. This is an **app-side shader** driven by a per-frame wind param (dir, strength, time)
   — the engine only needs a clean **hook to pass per-frame globals** (already possible via
   push-constants / a bound uniform; may add a small `time`/`wind` field to the view contract).
6. **Instanced vegetation placement.** Raw instancing exists; missing is a **scatter/placement**
   helper (grass/rock distribution over the ground) + **distance fade / density LOD** to keep
   grass cheap. Placement can live game-side; a reusable scatter helper could live in `core`.
7. **Ground material blending (grass ↔ dirt).** A blend/splat between two textured materials via
   a mask (or vertex-weight / height rule) — an **app shader** over a subdivided ground mesh.
8. **Frustum culling.** `scene::extract` currently buckets **all** instances with no culling.
   For a small arena this is minor, but grass instance counts get large ⇒ add per-view frustum
   cull in extraction to protect the character budget.
9. **Local atmosphere polish.** Mostly already covered (sky + height fog + aerial perspective);
   this milestone tunes them for the look. Volumetric sun shafts / bounded local fog volume are
   **stretch**, not required.

Not needed for the arena (explicitly deferred): CSM (single shadow map covers a small arena;
revisit if the sun-frustum can't cover it), skinned meshes + animation (characters, later),
non-flat terrain collision, SSAO/GI.

---

## Key architectural decision: engine mechanisms vs. game content

`goals.md` is explicit: **the engine builds no shaders/pipelines — the app supplies them**, and
this is a **library with no `main`**. That cleanly splits the work:

- **Engine (this repo) — the enabling mechanisms** that are missing and reusable:
  asset loading (`core::io`/`core::image`/`loadObj`), RHI **bindless textures + mip generation**,
  **vertex tangents**, the **richer material struct** + shader-side albedo/normal sampling in the
  stock `mesh.slang`, an **alpha-cutout material flag**, **frustum culling** in `scene::extract`,
  and a **per-frame wind/time hook** on the render contract.
- **Game (new repo) — the content + shaders**: the arena assembly (ECS world, camera, sun, fog,
  `GraphicsConfig`), the assets (ground/grass/dirt/bark/leaf textures, rock + tree meshes), and
  the **app pipelines/shaders**: ground grass↔dirt blend, the wind-animated grass + leaf shaders,
  and the pipeline builds (mesh/sky/shadow/tonemap/fxaa — porting the setup already proven in
  `tst/graphics/visual/*`).

So a lot of the "grass waving in the wind" is actually a *game-side shader* riding on a small
engine-side hook — not a big engine feature. The heavy engine lifts are **textures (bindless +
loading + mips)**, **tangents/normal maps**, and the **material upgrade**.

### Game repo shape

- New repo (e.g. `arena`), engine pulled in as a **git submodule** + `add_subdirectory`, linking
  `engine::{core,rhi,render,scene,ecs,input,input_glfw,controls,physics}`.
- Owns its **window + main loop** (the engine has none), builds its `.slang` → `.metallib` the
  same way `src/shaders/` does, and assembles the scene each frame → `RenderView`s → `Renderer`.
- Start it by lifting the pattern from `tst/graphics/visual/atmosphere_scene` / `shadow_scene`
  (camera entity + fly controller + sky/shadow/fog/AA toggles) into a standalone app.

---

## Phased plan (knock out one-by-one)

Standing rules: **performance-lean; benchmark before/after; test early + often; opt-in +
parity-preserving engine changes** (don't regress the existing graphics pixel tests). Each engine
phase lands with unit/integration + a visual test in this repo *before* the game repo consumes it.

**Phase 0 — Game repo scaffold + baseline lit scene.**
Stand up the new repo with the engine as a dependency; a windowed loop with a camera entity + fly
controller, a lit ground plane + a few boxes, and the existing sky + shadows + fog + MSAA wired
from app-built pipelines. Proves the library boundary + establishes the perf baseline. *No new
engine features.*

**Phase 1 — Asset + texture foundation (engine: `core` + `rhi`).**
- `core::io::readFile` + `core::image` (`Image` + `loadImage` via stb_image).
- `core::geometry::loadObj` (tinyobj; port from parked Vulkan `graphics_model.cpp`) → `ModelData`
  → `GeometryCatalog`; compute tangents at load.
- RHI: **implement the bindless texture table in the Metal backend** + a **`generateMipmaps`**
  path. *Decision:* go bindless (Metal argument buffer / heap) rather than a texture-array
  fallback — it's the intended design (`Device` API + shader already assume it) and scales to
  authored content. Keep a small explicit-binding path for post passes (already there).
- Add a **tangent** attribute to `core::Vertex`; update `render::coreVertexLayout()` + `mesh.slang`.
- **End-to-end proof**: a textured, mipmapped, normal-mapped box/quad + a pixel test.

**Phase 2 — Material upgrade + ground & props.**
- Extend `core::Material` + `render::MaterialGPU`: albedo + normal + ORM textures + roughness/
  metallic + emissive + **alphaCutout flag**; `mesh.slang` samples albedo + applies normal
  mapping (Lambert first; light PBR optional). Opt-in, parity anchors intact.
- Game: subdivided ground mesh with a **grass↔dirt blend** shader (mask/vertex-weight); scattered
  **rock** props (loaded meshes, instanced); a **tree** mesh (trunk + branches).

**Phase 3 — Vegetation + wind.**
- Verify/enable the **alpha-cutout + double-sided** foliage material path (discard + `CullMode::None`).
- Engine: a **per-frame wind/time hook** on the render contract (small field or documented
  push-constant convention); optionally a reusable **scatter/placement** helper in `core`.
- Game: grass-blade + leaf **wind shaders** (base-anchored bend + flutter, `stb_perlin`/time
  driven); instanced grass over the ground; leaves on the tree. **Distance fade / density LOD.**
- Benchmark grass instance-count sweeps; keep headroom for characters.

**Phase 4 — Atmosphere polish + performance pass.**
- Tune sky / height fog / aerial perspective for the local look; (stretch) sun shafts or a bounded
  local fog volume.
- Engine: **frustum culling** in `scene::extract`/`extractViews`; instance + draw benchmarks.
- (Only if needed) CSM if the single shadow map can't cover the arena cleanly.

**Phase 5 — (later, out of scope here) characters.**
Skinned mesh + animation, a character controller, and hooking the physics humanoid/fighters into
the arena. Tracked separately; the arena's job is to be ready (and fast enough) for them.

---

## Integration risks / watch-items

- **Vertex layout change (tangents)** touches every mesh pipeline + `mesh.slang`; do it once,
  deliberately, and re-run the graphics pixel tests (the `vertex.h` comment already anticipates
  this).
- **Bindless on Metal** is the riskiest single engine piece (argument buffers / residency,
  frames-in-flight lifetime). Keep the explicit-binding post path working alongside it.
- **Parity**: every material/shader change must stay opt-in so `graphics.*` pixel anchors hold.
- **Perf budget**: grass overdraw (alpha-tested cards) is the classic outdoor cost — lean on
  LOD/distance-fade + froxel lighting; benchmark before/after each vegetation step.
- **engine vs game placement**: resist putting arena-specific shaders/content in the engine —
  only the reusable mechanism lands here (per the goals split).
