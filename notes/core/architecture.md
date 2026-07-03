# Architecture (as-is)

Snapshot of the current structure. This describes what exists today, not the target.
Last synced with code: 2026-07-03.

## Build & layout

- C++23, CMake `>= 3.25`, `compile_commands.json` exported.
- Split `include/` (public headers) vs `src/` (implementation).
- Per-module build files live under a single top-level **`modules/`** dir (`modules/<name>/
  CMakeLists.txt`, aggregated by `modules/CMakeLists.txt`), separate from their source under
  `src/` and headers under `include/`. `tst/` (which holds actual driver sources) stays at the
  root. CMake uses `GLOB_RECURSE` (incl. `.mm`) + `source_group`.
- **Backend is platform-selected**: `if(APPLE)` → Metal + QuartzCore frameworks;
  `else()` → `find_package(Vulkan)`.

```
engine/
├── CMakeLists.txt              # top-level: deps + platform backend select + module wiring
├── include/engine/<module>/    # public headers per module
├── src/<module>/               # implementation per module
├── modules/                    # per-module build files only (no sources)
│   ├── CMakeLists.txt          # aggregator: add_subdirectory(core, ecs, graphics, ...)
│   ├── core/CMakeLists.txt         engine_core       (STATIC)
│   ├── ecs/CMakeLists.txt          engine_ecs        (STATIC)
│   ├── graphics/CMakeLists.txt     engine_graphics   (STATIC); Metal|Vulkan per platform
│   ├── scene/CMakeLists.txt        engine_scene      (STATIC); ecs↔render bridge
│   ├── physics/CMakeLists.txt      engine_physics    (STATIC)
│   └── physics_ecs/CMakeLists.txt  engine_physics_ecs(STATIC); physics↔ecs bridge
├── shaders/                    # .slang → .metallib/.spv via slangc
├── tst/                        # tests by <module>/<category>/ → tests | benchmarks | visuals
└── external/                   # submodules: glfw, glm, stb, tinyobjloader, metal-cpp
```

## Target graph

```
engine::engine (INTERFACE)
├── engine::core        (STATIC) → glm, tinyobjloader, stb, Threads
│                                  geometry (vertex/mesh/material/model/primitives),
│                                  memory/Handle, math/Transform, threading (ThreadPool + parallelSort)
├── engine::ecs         (STATIC) → engine::core   (archetype World, queries, resources, Schedule)
├── engine::graphics    (STATIC) → engine::core, glfw, glm, tinyobjloader, stb,
│                                  (Apple) Metal+QuartzCore | (else) Vulkan::Vulkan   (RHI + Metal backend)
├── engine::scene       (STATIC) → engine::ecs + engine::graphics   (ECS↔render bridge; extract)
├── engine::physics     (STATIC) → engine::core   (shapes/collision/broadphase/solver, PhysicsWorld)
└── engine::physics_ecs (STATIC) → engine::physics + engine::ecs    (RigidBody + step/sync systems)
```

Dependencies: `glm` via `find_package`; `glfw` + `tinyobjloader` via `add_subdirectory`;
`stb` header-only INTERFACE; `metal-cpp` vendored on the include path (Apple).

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

## Graphics module (`engine::graphics`)

**Current path (RHI + Metal)** — see the "Build reality (2026-07-03)" callout above for the
authoritative description. In short: a hand-written **RHI** (`include/engine/graphics/rhi/`) with
a working **Metal backend** (`src/graphics/metal/`), plus a **render layer**
(`include/engine/graphics/render/`) — `GeometryStore` (uploads `core::MeshData`), `RenderView`
(camera + `RenderItem[]` + per-instance `InstanceData[]` + `MaterialGPU[]`), and `Renderer`
(instanced `drawIndexed`, per-instance materials, depth, offscreen + windowed present). Slang
shaders (`shaders/*.slang` → `.metallib`). This is what the tests and the milestone exercise.

### Legacy Vulkan code (parked under `src/graphics/vulkan/`, NOT the current path)

The original monolithic `Graphics` class (~460 lines in `graphics.h`, derived from the
vulkan-tutorial.com flow) is parked pending the Vulkan-behind-RHI port (todo.md "Graphics
refactor pass" item 3). It does **not** build on Apple and is not linked into the Metal path.
Kept for reference / to port from:

- **`graphics.cpp`** (~48 KB) — instance/device selection, logical device, textures
  (staging, mipmaps, image views, sampler), buffers, single-time command helpers,
  memory-type lookup, debug messenger, validation layers, macOS portability handling.
- **`graphics_swapchain.cpp`** (~29 KB) — swapchain create/recreate/cleanup, render pass,
  descriptor set layout/pool/sets, the main graphics pipeline, command buffer recording,
  and the per-frame acquire → record → submit → present loop.
- **`graphics_custom.cpp`** (~27 KB) — reusable helpers for *custom/offscreen* targets:
  `createRenderPass`, `createFramebuffer`, `createDescriptorSet(s)`, `createPipeline`
  (incl. push-constant overload), and granular `record*` command helpers.
- **`graphics_model.cpp`** (~8 KB) — `loadQuad`, `loadObj` (via tinyobjloader).
- **`graphics_instance.cpp`** (~2 KB) — `updateGlobalUBO`, `addDrawJob`, `copyInstanceToBuffer`.

Legacy data structures (in the old `graphics.h`): `Vertex`/`Mesh`/`Material`/`GlobalUBO`/
`InstanceSSBO`/`DrawJob` + Vulkan helpers. Its shared-buffer + `DrawJob` + per-frame instance
SSBO layout was the data-oriented seed the current `render::` layer grew from. Hardcoded caps
still in that code: `MAX_FRAMES_IN_FLIGHT = 1`, `NUM_TEXTURES = 16`, `MAX_ENTITIES = 16`,
`WIDTH/HEIGHT 800x600`. Known smells to fix during the port live in todo.md.

## Physics module (`engine::physics`, Phase 0–2 + collision polish, 2026-07-03)

ECS-free, backend-agnostic core (depends on `engine::core` only — no ecs, no graphics).
Design + phasing + differentiable-backend design-ahead: investigations/2026-07-03-physics-plan.md.
- **Shapes**: `Sphere`, `Plane` (half-space), `Box` (oriented), `ConvexHull` (vertex set),
  `Capsule` (segment + radius). GJK `support()` seam via `SupportShape`.
- **Collision**: exact primitive fast paths (`sphereVsSphere`, `sphereVsPlane`, analytic
  `sphereVsBox`, `boxVsPlane`/`pointsVsPlane`/`capsuleVsPlane` multi-contact resting, analytic
  `capsuleVsSphere`/`capsuleVsCapsule`), **GJK closest-distance** (`gjk_distance`, witness
  points) for accurate **capsule-vs-box/hull** (segment↔convex, up to 2 contacts), **GJK + EPA**
  for the separating normal, and **face-clip manifolds** for polytope pairs — `box_box` and
  generic `convex_manifold` (box-hull, hull-hull) — giving up to 4 points so boxes/hulls
  **stack** stably. Verified: `tst/physics/unit/collision.cpp` (box/hull depth+normal exact,
  GJK-distance, capsule-convex), `tst/physics/unit/{sphere_box,manifolds,segments}.cpp`
  (sphere-box face/edge/corner/inside, plane manifold counts, capsule segment cases), and
  `tst/physics/integration/rigidbody.cpp` (box/hull/capsule rest flat |ω|≈0; sphere-on-box;
  box & hull **stacks** upright; capsule-on-box).
- **Solver robustness**: Baumgarte correction velocity is **clamped** (prevents fast/low-inertia
  bodies diverging). **CCD** (`WorldDef::continuousDetection`, default on): swept broadphase
  AABBs + **speculative contacts** (contacts within the substep's closing distance, solved so a
  body stops at the surface rather than tunnelling). The speculative branch targets the
  **restitution rebound velocity** when a pair is bouncing (so CCD doesn't brake the approach
  away and silently cancel restitution). Verified: a 120 m/s sphere stops on a small box
  (y=0.60) instead of passing through (y=−27), and an e=1 drop rebounds fully with CCD on
  (`tst/physics/integration/{rigidbody,dynamics}.cpp`).
- **Dynamics**: `RigidBodyState`, `PhysicsMaterial` (restitution, friction, compliance),
  inertia helpers; pure integration kernels — semi-implicit `integrateLinear`, SO(3) exp/log
  orientation (`so3ExpMap`/`integrateOrientation`), differentiable-ready (plan §14).
- **`PhysicsWorld` interface** (Phase 1): runtime-virtual coarse boundary (`createBody`/`step`/
  bulk `poses()`/`linearVelocities()`/`angularVelocities()`/`contacts()`), `BodyDef`/`WorldDef`,
  `createPhysicsWorld(Backend, ...)` factory. Multiple backends can coexist (plan §1).
- **Realtime backend** (`backends/realtime`, private to its TU): semi-implicit Euler +
  **sequential-impulse (PGS)** contact solver (restitution, Coulomb friction, Baumgarte),
  substeps × velocity iterations. Friction at the contact point applies torque ⇒ **true
  rolling**.
- **Broadphase** (Phase 2, `broadphase/`, selectable via `WorldDef::broadphase`):
  - **Sweep-and-prune** (single-axis): great for small/clustered scenes; ~O(n^5/3) for a 3-D
    cube of bodies (active list holds a whole slab).
  - **Uniform spatial-hash grid** (default): flat "index sort" (sorted `(cellHash,body)`
    entries + `thread_local` scratch, no per-step allocation), cell = largest AABB. **Linear
    scaling**. The entry sort uses `core::parallelSort` when the world has a pool (sparse 65k
    free-fall: ~1.3× from parallel sort + integration).
  Both verified against brute force in `tst/physics/unit/kernels.cpp`. Planes (infinite) are tested against
  finite bodies directly. Measured (Release, free-fall, `tst/physics/benchmark/step.cpp`): at **65,536**
  bodies grid = **19.9 ms/step** vs SAP 192 ms vs the O(n²) baseline's ~14 s extrapolation
  (~**700×**); grid throughput ~3.3–5.4 M body-steps/s across 256→65k (flat), SAP falls
  15.6M→0.34M. Crossover ~1–2k. 100k ≈ 30 ms/step single-threaded. See
  investigations/2026-07-03-physics-baseline.md.

### Threading (`engine::core::ThreadPool`)
A minimal fixed-size worker pool with a blocking `parallelFor` (dynamic work-stealing; the
caller thread participates). Verified by `tst/core/unit/thread_pool.cpp` (every index visited once, no
races). `core` links `Threads::Threads`. Two physics consumers:
- **Parallel worlds** (ML many-envs / "parallel simulations"): independent `PhysicsWorld`s
  stepped concurrently — **7.7× on 12 workers** (4.7M → 36.8M body-steps/s).
- **Intra-world** (optional `WorldDef::threadPool`): the step parallelizes integration,
  narrowphase (per-pair, lock-free), and the **contact solver via graph coloring** (same-color
  contacts share no dynamic body → solved in parallel; colors sequential). Both serial and
  pooled paths use the color order, so results are **bit-identical** (determinism verified in
  `tst/physics/integration/rigidbody.cpp`, max err 0.0). Dense 32k-body pile: **1.66×** (Amdahl-limited by the still
  serial grid sort + per-color barriers — a parallel sort is the next lever). Static bodies are
  never written, so shared colliders (the plane) are race-free.
- `physics::Real` localizes the scalar for a future double/dual-number switch.

### physics_ecs bridge (`engine::physics_ecs`)
Depends on `engine::physics` + `engine::ecs` (separate from `scene`, which pulls graphics).
`RigidBody{ BodyHandle }` component (no pose — Q2); `PhysicsWorldRef`/`FixedStep` resources;
`stepSystem` + `syncSystem` (bulk world poses → `Transform`) added to an `ecs::Schedule`.

### Milestone status — "ball rolling down a plane" ✅ (physics + sim)
- `tst/physics/integration/milestone.cpp` (headless): sphere on a 30° incline via the ECS bridge
  + scheduler. Verified it descends, travels down-slope, and **rolls without slipping** —
  |ω|·r ≈ down-slope speed, matching `a = g·sinθ/(1+2/5)` (measured 7.02 m in 2 s vs 7.01 analytic).
- `tst/physics/visual/rolling.cpp` (windowed, user-run, in the `visuals` runner): N spheres
  rolling down a tilted plane, physics → sync → `scene::extract` → Metal Renderer. Ties every
  subsystem together.
- **Test coverage** (`tst/physics/{unit,integration}/`): collision unit cases (sphere-box
  face/edge/corner/inside, plane manifold counts, capsule segment cases incl. degenerate/NaN
  safety, GJK/EPA), SO(3) exp/log + inertia, and integration cases (determinism, resting,
  stacking incl. 3-box, capsule, CCD, **restitution**, **friction**, **elastic collision**,
  **broadphase equivalence**, **kinematic motion**). Two bugs found + fixed 2026-07-03
  (kinematic bodies didn't move; CCD suppressed restitution) — see
  [2026-07-03-physics-test-findings.md](../investigations/2026-07-03-physics-test-findings.md).
- **Recent solver additions**: **kinematic bodies** advance by their scripted velocity (ignore
  gravity/impulses, still infinite-mass to the solver); **restitution works under CCD**.
- **Not yet (Phase 3, deferred)**: implicit/differentiable backend + parallel worlds as an
  ML training harness. Physics **Phase 2 + collision polish complete**: SAP + uniform-grid
  broadphase (parallel-sorted), ThreadPool (parallel worlds + colored solver + parallel sort),
  sphere/plane/box/hull/capsule colliders with GJK/EPA + GJK-distance + resting/stacking
  **manifolds**, clamped-Baumgarte solver, and **CCD** (swept AABBs + speculative contacts).
  Possible later: persistent warm-started manifolds, box-box/hull CCD (conservative advancement),
  joints/constraints, sleeping/islands.

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
  `tst/ecs/integration/scheduler.cpp` (gravity→integrate, hand-verified deterministic result).
- Not yet: command buffer for deferred structural changes, add/remove-component, parallel
  worlds / within-system parallelism (all planned).
- Driver: `tst/ecs/unit/entities.cpp` (spawn 1500 across 2 archetypes, query/mutate/chunk/destroy, verified).

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
- Driver: `tst/graphics/integration/scene.cpp` (ECS entities → extract → render → pixel-verified);
  `tst/graphics/visual/grid.cpp` is ECS-driven (a bob system + extraction each frame).

## What is NOT here yet

Genuinely open gaps (see todo.md for the full backlog):

- **ECS**: command buffer for deferred structural changes, add/remove-component, and parallel
  worlds / within-system parallelism. (Archetype core, queries, resources, and the ordered
  `Schedule` all exist.)
- **Vulkan behind the RHI**: only the Metal backend is live. The legacy Vulkan code is parked
  under `src/graphics/vulkan/` and not yet ported behind the RHI (graphics refactor item 3).
- **Headless/offscreen split**: the Metal `Device` has a headless path and offscreen tests, but
  `Swapchain` and `Renderer` aren't cleanly separated yet, and there's no window-free *windowed*
  path abstraction for ML/offline (graphics refactor item 7).
- **Bindless textures**: `baseColorTexture` + `Device::registerBindlessTexture` are stubs
  (per-instance material *colors* work; textured surfaces are the next step).
- **Compute**: no compute pipelines or compute-queue usage (needed for ML + some rendering).
- **`core::io`/`image`**: `readFile`/`loadObj`/`loadImage` + `Image` not yet in `engine::core`
  (loaders still live in the parked graphics code).
- **Physics Phase 3**: implicit/differentiable backend + parallel-world ML harness.
- **Application entry point**: intentionally none — this is a library; driver tests under
  `tst/` stand in for the consuming app.

Already done (previously listed here as missing): `engine::core` is populated (geometry,
memory/Handle, math/Transform, threading); the Metal backend has pipelines, command recording,
swapchain/present, and Slang shaders (`.metallib`/`.spv`); the ECS scheduler + resources +
`scene::extract` render-extraction all exist; and there's a real test harness
(`tst/harness/`, `tests`/`benchmarks`/`visuals` runners) with unit + integration coverage.
