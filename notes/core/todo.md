# TODO

Gaps between [goals](goals.md) and [current state](architecture.md), roughly prioritized.
Not commitments — a backlog to reason about.

## Foundational (blocks most other work)

- [~] **ECS core.** DECIDED archetype (2026-07-03); phase 1 landed: `engine::ecs` with
      `Entity` (generational `core::Handle`), `World`, archetype/table storage, and
      `query<Ts...>().each/.chunks`. Ships `engine::Transform` (in `core`). Verified by
      `tst/ecs/unit/entities.cpp`. **Resources + ordered scheduler DONE** (2026-07-03): `World::setResource/
      getResource` + `Schedule` (ordered `void(World&)` systems); `tst/ecs/integration/scheduler.cpp`
      (deterministic gravity→integrate). Next: command buffer + add/remove-component, and
      parallel worlds. **Render-extraction DONE** (2026-07-03):
      `engine::scene` bridge (`RenderMesh`/`RenderMaterial` components + `scene::extract` →
      `RenderView`); `tst/graphics/integration/scene.cpp` + ECS-driven `tst/graphics/visual/grid.cpp`. Plan:
      [2026-07-03-ecs-plan.md](../investigations/core/2026-07-03-ecs-plan.md).
- [x] **Driver test harness (not a `main`).** DONE (2026-07-03). The engine is a library with
      no application entry point; consuming apps own the loop. A self-registering harness
      (`tst/harness/`, `TST_CASE(module, category, name)`) drives subsystems; tests are organized
      `tst/<module>/<category>/*.cpp` and globbed into three runners — `tests` (unit +
      integration, CTest per module), `benchmarks` (Release), `visuals` (windowed demos). See
      goals.md "Testing / entry points".
- [~] **Break the window/swapchain coupling.** A headless device/context path so ML training
      and offline rendering run without GLFW. The **Metal `Device` has a headless path** (offscreen
      tests render + read back with no window). Remaining: cleanly split `Swapchain` from
      `Renderer` (graphics refactor item 7), and carry the same headless path through the Vulkan
      port. (The parked legacy Vulkan `Graphics` still assumes a window + swapchain end to end.)

## Active milestone: "a small, performant outdoor arena scene" (game-on-engine)

See goals.md + full plan: [2026-07-23-outdoor-arena-milestone.md](../investigations/realtime-rendering/2026-07-23-outdoor-arena-milestone.md).
A small outdoor arena — **grassy ground with dirt patches + small rocks, local atmosphere, a tree
with leaves + grass waving in the wind** — built as a **separate game repo** that takes this engine
as a dependency. First real game-on-engine (exercises the engine↔app split). Must be **performant**:
it's an arena for (later) fighting characters, so the static scene leaves a big budget for animated
characters. The **app supplies content + shaders** (grass/wind/ground-blend, pipeline builds); the
**engine supplies the missing mechanisms** below. Characters, non-flat terrain, networking are
deferred.

**Phase 0 — Game repo scaffold + baseline lit scene** (no new engine features):
- [ ] New repo (engine as git submodule + `add_subdirectory`); windowed loop, camera entity + fly
      controller, lit ground plane + boxes, existing sky/shadows/fog/MSAA from app-built pipelines
      (lift `tst/graphics/visual/atmosphere_scene`). Establishes the perf baseline.

**Phase 1 — Asset + texture foundation (engine: `core` + `rhi`):** ✅ DONE (2026-07-23)
- [x] **`core::io::readFile` + `core::image`** (`Image` + `loadImage`/`loadImageFromMemory` via stb_image).
      `include/engine/core/{io/io.h,image/image.h}` + `src/core/{io,image}`. stb impl compiled in
      `image.cpp` only; gated `ENGINE_ASSET_LOADERS` (training-only → invalid-Image stubs). Test
      `core.image_io_roundtrip` (self-contained TGA round trip).
- [x] **`core::geometry::loadObj`** (tinyobj → `ModelData`, faces grouped by material, corner-dedup)
      + **`computeTangents`** (per-vertex, Gram-Schmidt, ±1 handedness, degenerate-UV → w=0).
      `include/engine/core/geometry/obj_loader.h` + `src/core/geometry/obj_loader.cpp`
      (`namespace engine::geometry`). Test `core.obj_loader_quad`.
- [x] **RHI bindless texture table — implemented in the Metal backend** (`Device::registerBindlessTexture`/
      `unregisterBindlessTexture`, real slot table; `kMaxBindlessTextures=64`) + **`Device::generateMipmaps`**
      (blit) + **`CommandList::bindBindlessTextures(baseSlot)`**. Bounded texture-array bindless (Slang packs
      `materialTextures[64]` at fragment texture slots 1..64; shadow map stays at 0; material sampler at
      sampler slot 1) — no argument buffers/residency needed, fully controllable. Explicit single-texture
      binding kept for post passes.
- [x] **Tangent attribute on `core::Vertex`** (`glm::vec4 tangent`, w=handedness) + `render::coreVertexLayout()`
      attr loc 4 + `mesh.slang` VSInput. Vertex-layout change verified parity-preserving (all graphics pixel
      tests green).
- [x] End-to-end proof `graphics.bindless_textured`: albedo bindless sampling (exact texel), tangent-space
      normal mapping (flat R=181 → tilted R=213), mipmap generation (minified checkerboard → gray 128), all
      through the real `mesh.slang` + `Renderer` (opt-in via `RenderResources.materialSampler` +
      `MaterialGPU.baseColorTexture`/`normalTexture`, `-1` defaults keep parity). Benchmark
      `graphics.bindless_textures` (upload+mipgen throughput; textured draw ≈ untextured, negligible overhead).
      Full suite 188/0; training-only build still compiles (gated loaders).

**Phase 2 — Material upgrade + ground & props:**
- [x] Extended `core::Material` + `render::MaterialGPU` (metallic-roughness workflow) — DONE (2026-07-23):
      albedo + normal + **metallic/roughness (+ MR texture, glTF G/B pack)** + **emissive (factor + texture)**
      + **occlusion texture** + **alpha-cutout flag (`MaterialFlagAlphaCutout`) + `alphaCutoff`**. `mesh.slang`
      samples all of them, does alpha-cutout `discard`, applies AO to ambient, and adds a **gated Cook-Torrance
      GGX sun specular** (energy-conserving diffuse ×(1−metallic)). All gated so a default material stays pure
      Lambert ⇒ existing pixel tests byte-identical. GPU layout reflection-checked (80B scalar pack). `loadObj`
      fills baseColor/emissive/metallic/roughness/alpha from the .mtl. Test `graphics.material_features`
      (emissive-unlit, glossy peak > Lambert, cutout on/off). Suite 189/0; training-only core still builds.
- [ ] (game, deferred with Phase 0) Subdivided ground mesh with a **grass↔dirt blend** shader (mask/vertex-weight);
      scattered **rock** props (instanced loaded meshes); a **tree** mesh.

**Phase 3 — Vegetation support (engine mechanisms; wind MODEL is game-side)** ✅ DONE (2026-07-23)
> Design correction (2026-07-23): a *specific wind model* is content, not engine — it contradicts
> "the app supplies shaders." So the engine provides the general *mechanisms* to build/route a game's
> foliage shader; the wind look lives in a game shader. (An earlier pass had baked wind into the stock
> `mesh.slang`; that was reverted.)
- [x] **General material features** (kept, content-agnostic): **alpha cutout** (`MaterialFlagAlphaCutout`
      + `alphaCutoff`, discard) + **two-sided lighting** (`MaterialFlagDoubleSided` → flip the normal to
      face the viewer on back faces via `SV_IsFrontFace`). Test `graphics.two_sided_lighting` (back face
      0→765 lit).
- [x] **Pipeline factory** `Renderer::createMeshPipeline(MeshPipelineVariant, targetColorFormat)` — the
      engine owns the forward-pass contract (vertex layout, HDR/FXAA/MSAA-aware color format, depth format,
      sample count, bindings); the app passes only its **compiled Slang shader blob** + intent knobs
      (`cull`/`blend`/`depthWrite`/`depthCompare`). So a game builds opaque/foliage/overlay variants with
      **no backend (Metal/Vulkan) code and no reverse-engineering** the pass formats.
- [x] **Per-item pipeline selection** — `RenderItem.pipeline` (invalid ⇒ default `RenderResources.mesh`);
      forward pass binds per item so opaque + foliage/terrain variants coexist in one view. Test
      `graphics.pipeline_variants` (far item: default pipe ⇒ depth-occluded green; overlay variant ⇒ red).
- [x] **Shared Slang include** `shaders/engine_mesh.slang` — the binding/vertex contract (Globals/Instance/
      Material/PointLight/VSInput/VSOutput/bindless table) + `standardVertex()` + `shadeSurface()` helpers;
      `mesh.slang` is now a thin include+entry-points shader. Exposed via `ENGINE_SHADER_INCLUDE_DIR` so a
      game foliage shader `#include`s it and reuses the contract (adds only its wind vertex math).
- [x] **Reusable CMake shader helper** `cmake/EngineShaders.cmake` `engine_compile_shaders(TARGET/OUT_DIR/
      SOURCES/INCLUDE_DIRS/DEPENDS)` — slangc → metallib/spv with backend selection + include-dependency
      tracking; the engine's `src/shaders` uses it and consumers include the module. Suite 191/0; training-only builds.
- [x] **Honor `RasterState.cull` in the backend — DONE (2026-07-23).** The Metal backend now applies
      `setCullMode` + `setFrontFacingWinding`. Root cause of the earlier breakage: the primitives were
      inconsistently wound (`makeSphere`/`makeCapsule` CW-outward; `makeBox` `±Y` faces CW-outward) —
      fixed so ALL geometry is **CCW-outward** (the engine convention), after which the winding maps 1:1
      (`FrontFace::CounterClockwise` → `MTL::WindingCounterClockwise`, no Y-flip). `createMeshPipeline`
      defaults to `cull=Back`; fullscreen passes were already front-wound. Full parity held + dedicated
      `graphics.backface_cull` test (front drawn, back culled, `cull=None` draws both). Suite 195/0. Now
      `CullMode::None` on foliage genuinely draws both sides. Full write-up:
      [2026-07-23-backface-culling-winding.md](../investigations/realtime-rendering/2026-07-23-backface-culling-winding.md).
- [ ] (game, deferred with Phase 0) grass-blade + leaf **content/meshes + the wind shader** (a foliage
      `.slang` that `#include`s `engine_mesh.slang`, adds height-weighted sway, built via the factory as a
      cull-none/cutout variant and routed per-item), instanced grass over the ground, leaves on the tree,
      and **distance fade / density LOD** + grass instance-count benchmark.

**Phase 4 — Atmosphere polish + performance pass:**
- [ ] Tune sky/height-fog/aerial-perspective for the local look; (stretch) sun shafts / bounded local
      fog volume.
- [x] **Frustum culling** — DONE (2026-07-23). Core math `engine/core/math/bounds.h` (`core::Aabb` with
      `transformed(mat4)`; `core::Frustum::fromViewProj` Gribb-Hartmann for 0..1 clip z + AABB/sphere p-vertex
      tests) + `core::computeBounds(MeshData)` (`geometry/bounds.h`). `scene::cullToFrustum(frustum, in, span<Aabb>
      localBoundsPerItem, out)` compacts an ExtractedScene to instances whose world AABB (local box ×
      InstanceData.model) intersects the frustum; `scene::viewFrustum(view)` helper. Opt-in + lossless
      (empty bounds ⇒ passthrough); `extract()` unchanged (parity). Tests: `core.aabb_transform`,
      `core.frustum_cull`, `graphics.frustum_cull` (1/5 kept + full-vs-culled 0 pixel diffs). Benchmark
      `graphics.frustum_cull` ≈ 58–60 M instances/s (single-threaded, O(N)). End-to-end WITHOUT vs WITH
      culling scales with per-object GPU cost: trivial 12-tri boxes 1.1–1.9× (cull-bound, savings ≈ cull
      cost) but heavy ~2.3K-tri meshes **30–34×** (20k spheres 30 ms → 1 ms) — a big win for real meshes;
      the naive linear cull only bottlenecks at huge counts of trivial geometry (→ parallelize/coarse-cull
      later). Suite 194/0. (BVH / hierarchical cull is the later scaling step.)
- [ ] (only if needed) CSM if the single directional shadow map can't cover the arena.
- [ ] **(follow-up, perf) parallelize / coarse-cull `cullToFrustum`** — the current cull is single-threaded
      O(N); it becomes the bottleneck only at huge counts of trivial geometry (see the benchmark). Options:
      `ThreadPool::parallelFor` over instances, and/or a coarse grid/BVH so not every instance is tested.
      Deferred (not needed for the arena's instance counts).

**Phase 5 — (later, out of scope) characters:** skinned mesh + animation, a character controller,
hook the physics humanoid/fighters into the arena.

## Driving milestone: "a ball rolling down a plane"

See goals.md. Long-term vertical slice: sphere rolling down an inclined plane, plus the
capability targets that make it exercise our goals — headless + deferred rendering, parallel
sims for training, and massive scale (~100k spheres). This is the yardstick for whether core
is "good enough" and defines what the graphics refactor pass must support.

## Next milestone: "a physics humanoid walking on terrain" (RL-ready)

See goals.md + full plan: [2026-07-03-humanoid-rl-milestone-plan.md](../investigations/physics/2026-07-03-humanoid-rl-milestone-plan.md).
Articulated, actuated humanoid on procedural terrain, drivable by keyboard OR an action vector,
lit + steppable headless in parallel batches with batched obs/action tensors. Engine-side
mechanism only — reward/RL-algorithm/cloud/task live in a **downstream sim repo** (see goals.md
"engine ↔ simulation split"). Big open decision: **articulation approach** (maximal-coordinate
joint constraints vs reduced-coordinate Featherstone) — needs its own design doc before Phase B.

### Phase A — Interaction + graphics basics ✅ DONE (2026-07-03)
Backend-agnostic input + ECS-native camera. Split by dependency for extensibility (GLFW today,
Qt/other later) and headless/ML use:
- [x] **`engine::input`** (INTERFACE, GLFW-free): `InputState` (level + edge queries, mouse
      pos/delta incl. `addMouseDelta` for event backends, scroll) + `Key`/`MouseButton`. Adapter
      contract fits polling (GLFW) + event-driven (Qt) + scripted/ML. `tst/input/unit`.
- [x] **`engine::input_glfw`** (adapter): `GlfwInput` polls a window (`void*`) → `InputState`;
      scroll callback; cursor capture. Future `engine::input_qt` sits alongside.
- [x] **`engine::controls`** (ecs + input + core, graphics-free): `FlyController` component +
      `flyControllerSystem` (reads `InputState` + `Time` resources → drives `Transform`).
      `tst/controls/unit/fly_controller.cpp`.
- [x] **Camera as an ECS entity**: render-agnostic `engine::Camera` projection in `core::math`;
      a camera entity = `Transform` + `Camera` (+ optional `FlyController`). `scene::extractViews`
      turns `<Transform, Camera>` → `RenderView`. `engine::Time` frame-dt resource in core.
      `tst/core/unit/camera.cpp`.
- [x] **Lighting**: `render::DirectionalLight` on `RenderView`; renderer packs `GlobalUniforms`
      (viewProj + light) into binding 0; `mesh.slang` does ambient + Lambert (configurable).
- [x] **Background**: `scene::Background` + `scene::SceneLighting` resources + `applyEnvironment`.
      Demo: `tst/graphics/visual/input_demo.cpp` (camera entity + Schedule[input-pump → fly] +
      extractViews + background/light toggles).

### Phase B — Articulated physics (the core; long pole)
- [x] **B0 design doc**: DECIDED (2026-07-03) —
      [2026-07-03-articulation-approach.md](../investigations/physics/2026-07-03-articulation-approach.md).
      **Maximal-coordinate joint constraints first** on the existing impulse solver; add a
      **reduced-coordinate (Featherstone) `PhysicsWorld` backend later** (deferred, but integral
      once training starts — smaller observations → throughput). Keep joint/actuator + obs/action
      API backend-agnostic.
  Full reviewed plan (grounded in the solver + checked vs core goals) in the milestone plan
  §"Phase B — APPROVED PLAN". Joints = persistent velocity constraints in the same solver loop
  as contacts, warm-started across steps; **per-world, allocation-free, no statics** (VecEnv-safe).
- [x] **B1 Joint constraints core** ✅ (2026-07-03): `JointHandle`/`JointType{Ball,Revolute,Fixed}`/
      `JointDef` + `createJoint`/`destroyJoint` in `world.h`; persistent `joints_` store +
      serial warm-started solve (ball = point-to-point 3×3 K; hinge = point + 2 angular; fixed =
      point + 3 angular lock) + Baumgarte. Tests: ball anchors coincide, hinge off-axis ≈ 0,
      fixed keeps relative pose.
- [x] **B2 Joint limits** ✅ (2026-07-03): hinge min/max angle as a one-sided constraint about
      the axis. Test: pendulum settles exactly at the clamp (`hinge_limit_stops`).
- [x] **B3 Actuators + state read** ✅ (2026-07-03): per-Revolute {Torque, PDTarget(kp,kd,target,
      targetVel)} + `maxTorque` (external angular impulse pre-solve). Read `q/qd` single
      (`jointState`) + **bulk SoA span** (`jointStates`); **bulk SoA actuator write**
      (`setJointTargets/Torques`) + single setters. Tests: PD holds a target angle vs gravity,
      torque drives the joint, bulk write/read roundtrip. Ball/Fixed actuation + multi-DOF q/qd
      deferred to B4 (humanoid hips/shoulders).
- [x] **B4 ECS bridge + builder + humanoid preset + self-collision filtering** ✅ (2026-07-03):
      `collisionCategory`/`collisionMask` on `BodyDef` (limbs mask each other out); plain-data
      `ArticulationDef`/`JointSpec` + `buildArticulation` + `makeHumanoid` preset (13 limbs,
      12 joints) in `physics/dynamics/articulation.h`; 3-DOF ball spherical-PD/torque actuation;
      `WorldDef::linear/angularDamping` (drag so free DOFs settle); ECS `Joint`/`JointCommand`
      components + `actuatorFlushSystem` + `spawnArticulation` bridge. Tests: collision_filter,
      ragdoll_settles, ball_actuator_holds_orientation, articulation_ecs. Flat per-joint ball
      q/qd SoA for RL obs deferred (derivable from body poses) until the downstream obs format.
- [x] **B5 Tests + demo** ✅ (2026-07-03): `ragdoll_settles` (passive ragdoll collapses + rests,
      |v|,|ω|→0, no explosion); `pd_stand_holds_pose` (pinned pelvis, PD drives bent knees vs
      gravity + holds neutral elbows); `articulation_determinism` (serial vs parallel bit-identical
      with joints+actuators, err 0); visual `humanoid` (PD-held humanoid on the plane, boxes +
      fly camera). Added `primitives::makeBox` + `WorldDef` damping. Engineering notes (solve
      order, inertia-ratio stiffness, explicit-PD-on-soft-chains) in the articulation note.

**Phase B (articulated physics) COMPLETE** ✅ — maximal-coordinate joints (ball/hinge/fixed) +
limits + actuators (revolute PD/torque + ball spherical PD) + q/qd read + bulk SoA action/obs +
self-collision filtering + articulation builder/humanoid preset + ECS bridge, all deterministic
and green. Next: **Phase D** (env interface) on the flat plane; reduced-coordinate Featherstone
backend + terrain (Phase C) remain deferred.

### Phase C — Terrain ⏸ DEFERRED (2026-07-03) — revisit after B/D
Deferred: a flat `Plane` is enough to get a humanoid balancing + walking; varied terrain is a
refinement, not a prerequisite. Design + rationale + the "GJK/EPA ≠ arbitrary mesh collider"
analysis preserved in
[2026-07-03-terrain-collision-deferred.md](../investigations/physics/2026-07-03-terrain-collision-deferred.md).
Revisit when locomotion needs slopes/stairs/rough ground (e.g. an RL terrain curriculum).
- [ ] (deferred) Heightfield collider (sphere→capsule; AABB-reject broadphase like Plane;
      analytic surface normal to dodge the internal-edge problem).
- [ ] (deferred) Procedural terrain generation in `core::geometry` → heightfield + render mesh.
- [ ] (deferred) Render terrain as a lit static mesh; body-settles-on-terrain test.
- [ ] (later, separate/larger) General concave triangle-mesh collider (convex decomposition, or
      per-triangle + BVH + internal-edge filtering). Convex meshes already work via `ConvexHull`.

### Phase D — RL-ready env interface (on the flat plane) ✅ DONE (2026-07-04)
Reviewed plan: [2026-07-04-phase-d-plan.md](../investigations/physics/2026-07-04-phase-d-plan.md). Headless
`Environment` drives a `PhysicsWorld` **directly (ECS-free)** in the new `engine::physics_env` module.
- [x] **D0 Solver perf micro-opt**: per-body world inverse inertia cached once per substep
      (`computeWorldInvInertia`), read by contacts + joints + actuators + limits. Bit-identical
      (determinism tests still 0.0); win scales with contacts×iterations.
- [x] **D1 `Environment`** (`engine::physics_env`, ECS-free, mirrors `physics_ecs`): `reset(seed)`
      in-place / `setAction` (Torque; actDim=21) / `step` + raw-state accessors (q/qd, root pose+twist,
      contact flags); obs composition downstream + optional default packer (obsDim=53). PhysicsWorld
      gained setBodyState/clearState/refreshState/setJointBallTorque. Tests: dims, bounded rollout,
      deterministic (0.0).
- [x] **D2 `VecEnv`**: N single-threaded worlds on the `ThreadPool` (`parallelFor`, no nesting);
      contiguous SoA `actions()[N×actDim]`/`observations()[N×obsDim]`; reset/resetMasked/step. Test:
      parallel == serial, bit-identical (0.0) at N=24.
- [x] **D3 Determinism + throughput**: single-env + parallel-vs-serial determinism both 0.0;
      benchmark `physics_env.vec_env_throughput` — N=1→1024 ≈ 8.7k→68.7k env-steps/s (13 workers).
      **Engine now has "enough" for the downstream RL repo** (env + batched obs/action).

**Phase D COMPLETE** ✅ — RL-ready mechanism (ECS-free `Environment`/`VecEnv`, in-place reset, raw
batched state + Torque actions, parallel-worlds throughput, determinism) on the flat plane.

### Phase E — Reduced-coordinate Featherstone backend (own track, after D)
Second `PhysicsWorld` (`Backend::Reduced`) behind the same Environment/obs-action API; the largest
single piece of the milestone, slip-able so it doesn't gate RL-readiness. Decision:
[articulation-approach.md](../investigations/physics/2026-07-03-articulation-approach.md).
- [x] **E0** ABA spatial-algebra core (no contacts); validate a free chain/pendulum vs invariants.
      DONE (2026-07-04): `Backend::Reduced` + `src/physics/backends/reduced/featherstone_world.cpp`
      (fixed + floating base, revolute/fixed joints, explicit per-link gravity, 6×6 spatial algebra,
      semi-implicit Euler). Validated in `tst/physics/integration/reduced.cpp`: pendulum period 0.5%,
      double-pendulum energy drift 0.54%, floating free-chain linear/angular momentum 0.1%/0.24%.
      Design + results: [reduced-coordinate-backend.md](../investigations/physics/2026-07-04-reduced-coordinate-backend.md).
- [x] **E1** contact coupling (contact-space inertia + PGS). DONE (2026-07-04): CRBA joint-space
      inertia `H`, generalized contact Jacobians (ancestor revolute cols + floating-base 6 cols),
      sequential-impulse **PGS in generalized coords** (Δq̇ = H⁻¹Jᵀλ; normal λ≥0 + Baumgarte +
      Coulomb friction), dynamic-link colliders (sphere/box/capsule) vs static planes. Validated:
      sphere settles at its radius (no penetration, v→0), box holds on a 17° slope at μ=1 and slides
      at μ=0.05 (friction cone genuine). CRBA cross-checked via the KE identity.
- [x] **E2** humanoid + actuators: **Ball (3-DOF) joints**. DONE (2026-07-04): unified multi-DOF
      rotation-joint model (relRot = restRel·locRot; per-axis S = [axis; −axis×anchorC]) generalizes
      ABA/CRBA/Jacobian/integration from revolute to ball; provably reproduces revolute (E0/E1 stayed
      green). Ball actuation (per-axis torque + spherical PD via quaternion error). `makeHumanoid`
      (14 bodies/13 joints, floating pelvis) runs on `Backend::Reduced`: ragdoll settles on the
      ground (bounded, at rest, no penetration); suspended humanoid holds its neutral pose via PD.
      Tests: ball-pendulum energy 0.04%, ball actuation, humanoid ragdoll, humanoid PD-hold.
- [x] **E3** behind `VecEnv`. DONE (2026-07-04): humanoid `Environment`/`VecEnv` run on
      `Backend::Reduced` by flipping `EnvConfig.backend` only (API unchanged, actDim=21/obsDim=53);
      finite random-torque rollout; single-env determinism + VecEnv parallel==serial bit-identical;
      throughput benchmark reduced vs maximal (~2× slower/step — dense H⁻¹ contact solve dominates;
      reduced needs finer substeps under strong torque). **Phase E COMPLETE.**
- [x] **Phase F — differentiable reduced env (hybrid α-order).** Primary research objective — **engine
      side COMPLETE** (2026-07-04). Review + A/D/hybrid comparison + plan: [2026-07-04-differentiable-reduced.md](../investigations/physics/2026-07-04-differentiable-reduced.md).
      Build (A) analytic differentiable step as the core, (D) zeroth-order over the fast VecEnv as the
      baseline, converge on the α-order hybrid. Forward-mode duals → per-step Jacobian at the env boundary.
  - [x] **F1** differentiable smooth dynamics (no contact) — **COMPLETE** (2026-07-04): `Dual`
        forward-mode AD + Scalar-generic linalg + generalized ABA (`include/engine/physics/diff/{dual,
        linalg,articulated}.h`) for fixed/floating base + revolute/ball/fixed joints, SO(3) exp-map
        integration, quaternion-free. Validated physically (period, energy, momentum) AND for exact
        gradients (revolute/ball/floating all match central FD to 8 digits). Next: per-step
        state/observation Jacobian at the env boundary.
  - [x] **F2** smoothed/compliant contact — **COMPLETE** (2026-07-04). F2a: compliant normal contact
        folded into the generic ABA + `softplus`/`sigmoid` on `Dual`. F2b: regularized Coulomb friction
        (contact-point slip) + radius-lever torque ⇒ sphere rolls without slipping (ω_z=−vx/r); coupled
        hopper (floating base + revolute leg + foot) gradient == FD to 8 digits; soft-contact bias =
        `r−mg/k` (tunable). All gradients through contact validated vs finite differences.
  - [x] **Fd** zeroth-order estimator — **DONE** (2026-07-04): `include/engine/physics/diff/zeroth_order.h`
        (deterministic antithetic Gaussian-smoothing ES gradient). Unbiased on a quadratic (variance ↓ with N);
        agrees with the analytic `Dual` gradient to 0.69% on the smooth pendulum. Both hybrid gradient sources
        now exist. N evals map onto the fast VecEnv downstream.
  - [~] **F3** α-order hybrid blend + per-step Jacobian/VJP at the env boundary + SHAC smoke test.
        **F3a DONE** (2026-07-04): `include/engine/physics/diff/hybrid.h` `alphaOrderGradient` (min-variance
        blend of analytic first-order + zeroth-order). Smooth pendulum ⇒ α=1.0, blend==analytic; stiff
        near-step (β=200) ⇒ α=0.15 + 4.5× lower across-seed variance than pure first-order (Suh et al.).
        **F3b next**: per-step state/observation Jacobian (tangent coords) at the env boundary / `DiffEnvironment`.
  - [x] **F3b Jacobian DONE** (2026-07-04): `include/engine/physics/diff/jacobian.h` `stepJacobian` — per-step
        tangent Jacobian `∂s_{t+1}/∂(s_t,a_t)` via exact forward-mode `Dual<1>` per column; orientation via
        exp/`vee`-log. Full 14×15 floating-base+revolute Jacobian == tangent-space FD to 4e-11 (all blocks).
  - [x] **F3c DONE** (2026-07-04): `from_articulation.h` (`articulationToDiffModel`) + `diff_environment.h`
        (`DiffEnvironment`: reset/setAction/step, `jacobian()`, `rolloutGradient<NA>()`). Real humanoid
        converts (14 links/21 DOF/floating, authored poses reproduced), runs finite/bounded, 54×75
        Jacobian finite, analytic rollout gradient == FD to 1.3e-9. **Phase F engine-side COMPLETE.**
  - [ ] (downstream) SHAC-style short-horizon analytic-policy-gradient smoke test — lives in the RL repo
        (reward/policy/optimizer) consuming `DiffEnvironment` + `alphaOrderGradient`.
  - [x] **Bug-hunting/hardening round DONE** (2026-07-04): unit `diff_invariants.cpp` + integration
        `diff_validation.cpp` + benchmark `diff.cpp` + visual `diff_humanoid.cpp`. Verified converter ==
        production reduced backend (9.2e-8 m), contact gradient == FD (3.7e-10), COM-ballistic, determinism.
        Fixes: **softened contact defaults** (groundK 3e3→2.5e3, C 30→80, β 800→120) + **`DiffEnvironment`
        auto substeps** (contact 48 / free 16) ⇒ passive humanoid drop stable at all substeps. Perf: diff
        forward 0.70× reduced (faster); rolloutGradient ~linear in seeds (NA=21 ≈50× fwd); Jacobian 11 ms.
        Note: [2026-07-04-differentiable-reduced.md](../investigations/physics/2026-07-04-differentiable-reduced.md)
        "Bug-hunting / hardening round".
  - [x] **Contact geometry + stability (F2/F3/F4) DONE** (2026-07-04), plan+results:
        [2026-07-04-differentiable-contact-geometry.md](../investigations/physics/2026-07-04-differentiable-contact-geometry.md).
        (2) multi-point contact mechanism; (3) shape-aware points (capsule caps / box corners, `DiffContact::{None,Feet,All}`);
        (4) semi-implicit contact behind `ContactIntegration` switch. Feature 3's multi-point contact conditioned the
        humanoid so the explicit path is stable to k=8e4 ⇒ restored groundK 2.5e3→1e4 (contact penetration ~0, ragdoll
        rests cleanly), explicit default, semi-implicit available for harder regimes. Full IMEX (M4) not needed.
- [x] **Physics config system (P1–P3) DONE** (2026-07-04), plan+results:
      [2026-07-04-physics-config-system.md](../investigations/physics/2026-07-04-physics-config-system.md).
      Centralized `SimConfig` (`include/engine/physics/config.h`) — un-buried the 10 solver constants
      (`SolverConfig`), `WorldDef`/`EnvConfig` derive from it (no duplication); `SimConfigOverride`+
      `resolve` sparse override layering + `configs.h` named presets; write-only `serialize`/`configHash`/
      `dump`+`configVersion` (`config_io.h`) for run history. Value-type, no global singleton. Next steps ↓.

### GPU / CUDA readiness (end goal: CUDA **alongside** CPU — CPU+Metal on Mac, CPU+CUDA on Linux/NVIDIA)
**Picking this up? Start here:** [2026-07-08-cuda-port-handoff.md](../investigations/physics/2026-07-08-cuda-port-handoff.md)
(ordered work plan: benchmark first, then the features left to implement).
Review of what a CUDA port needs: [2026-07-06-cuda-port-review.md](../investigations/physics/2026-07-06-cuda-port-review.md)
(target = AWS A10G `sm_86`; port the **reduced + smoothed-contact ABA, one-env-per-thread batched**;
reuse the scalar-generic `diff/` math; keep the maximal solver + general GPU collision out of scope).
- [x] **Baseline measured + blockers 2 & 3 resolved** (2026-07-08), CPU-only device-prep, all diff tests green.
      Note: [2026-07-08-cuda-port-blockers-fixed-size-flat-model.md](../investigations/physics/2026-07-08-cuda-port-blockers-fixed-size-flat-model.md).
      Baseline (M3 Pro): diff forward substep 0.0026 ms/385k/s. **Blocker 2** — `diffForwardDynamics` +
      `Accel` fixed-size-ified (`kMaxLinks=16`/`kMaxDof=32` stack arrays, ~12 per-call `std::vector`s gone,
      no caller ripple): heap-free hot path, substep → 0.0023 ms/431k/s (+12%), gradient at parity.
      **Blocker 3** — new `diff/flat_model.h`: POD trivially-copyable SoA `FlatModel` + `flatten(DiffModel)`
      (baked once, contact reps normalized, joint damping pre-resolved; 9232 B); fidelity-tested on
      humanoid+AMP (`physics.unit.diff_flat_model`). Suite 180/0.
- [x] **Per-env state fixed-size DONE** (2026-07-08) — `DiffState<S>` now fixed-size POD (`M3 linkRot[kMaxLinks]`,
      `S qd[kMaxDof]` + `numLinks/numDof`), trivially-copyable; the IMEX path is heap-free too
      (`computeContactForcesWorld` → fixed `ContactForces`, `linkWorldInto` writes a caller array,
      `extContactWorld` a plain pointer, the trial-state copy is now a memcpy). **The entire `diffSubstep`
      (Explicit + IMEX) allocates nothing.** `linkWorld` keeps a std::vector wrapper for host readout only.
      Suite 180/0; substep ~0.0025 ms (parity), gradient parity.
- [ ] **Batched one-env-per-thread CUDA kernel** consuming `FlatModel` + fixed-size `DiffState`; `Backend::Cuda`
      in `config.h`; `enable_language(CUDA)` + `-DENGINE_CUDA=ON` gated `if(NOT APPLE)`; float forward /
      double grad-check; tolerance-based CPU↔GPU parity tests (run on the Linux box). Precondition for the
      payoff: the PyTorch-CPU→GPU switch (obs/actions stay in VRAM).
- [ ] **Alignment**: RL env defaults to `Backend::Realtime` (maximal) but the port targets reduced/diff —
      confirm humanoid RL runs the reduced + smoothed-contact dynamics so CPU and CUDA are the *same* physics.

### Config + downstream / training bring-up (next)
- [ ] **Config: wire `dump()` into the training/VecEnv loop** — log each run's resolved config + hash
      with its results (the actual reproducibility payoff). Gated on the training loop existing.
- [ ] **Config: cross-engine unification** (deferred; user OK'd the two-surface split 2026-07-04) — fold
      the diff engine's contact knobs (`DiffModel`: `groundK/C/beta/mu`, `contactIntegration`) into a shared
      `ContactConfig` in `config.h` so ONE config covers both the RL and differentiable sims. Touches the
      diff engine + layering; do when a single cross-engine tuning/serialization surface is needed.
- [ ] **Config: key/value READER** — parse text → `SimConfig` for launch-time overrides / sweeps without
      recompiling; add when the training launcher consumes config files (the writer already defines the format).
- [~] **Python bindings** — P1 binding **drafted in `sim-1/csrc/`** (2026-07-04): nanobind `engine_py`
      exposing the ECS-free surface — `VecEnv` (zero-copy `actions()`/`observations()` NumPy views),
      `SimConfig` (+ `dump()`/`config_hash()`), `make_humanoid`/`make_amp_humanoid`, `EnvConfig` — plus a
      Python adapter (`sim1/envs/engine_vecenv.py`) presenting the `VecEnv` contract (slices the obs layout
      `pos3|quat4|linvel3|angvel3|q[ndof]|qd[ndof]|contacts[nbody]`) and mapping Python `EnvConfig` →
      engine `SimConfig`. scikit-build-core + `ENGINE_SOURCE_DIR` escape hatch wired.
      **BLOCKED on: (1) bumping the sim-1 engine submodule** to a commit with `config.h` + `makeAMPHumanoid`
      (`caa5256` not yet pushed to origin), **(2) `nanobind`/`scikit-build-core` install + a C++ compile pass.**
      SHAC/`DiffEnvironment` binding is a later add. Not compiled yet.
- [~] **Adopt a richer humanoid model for mocap training** — plan:
      [2026-07-04-humanoid-rig-adoption.md](../investigations/physics/2026-07-04-humanoid-rig-adoption.md).
      **Decision:** rig-agnostic engine, support BOTH rigs (existing 21-DOF `makeHumanoid` + AMP 28-DOF
      `makeAMPHumanoid`, SMPL later) — rigs are `ArticulationDef` data, selectable per experiment; only
      trained weights + retarget config are per-rig.
      **DONE (2026-07-04): rig-agnostic per-joint physics features** in the diff engine (`DiffLink`):
      per-joint `jointDamping` (<0 inherits the model global), passive `jointStiffness` (smooth `vee`
      spring toward rest), and `armature` (rotor inertia on the joint-space diagonal). Tests
      diff_joint_damping_per_link / _stiffness_restores_to_rest / _armature_adds_rotor_inertia /
      _stiffness_differentiable (146/0). Defaults are no-ops (existing models unchanged).
      **DONE (2026-07-04): `makeAMPHumanoid()`** authored (15 bodies / 28 DOF AMP topology, Y-up, feet-last)
      in `articulation.cpp`; validated headless (`tst/physics/integration/amp_humanoid.cpp`: 15 bodies /
      28 DOF / floating, converts to DiffModel, passive drop rests on the plane) + windowed visual
      `tst/physics/visual/amp_humanoid.cpp` (passive ragdoll via the diff engine, per-collider meshes).
      **TODO (deferred):** faithful mass-from-density + per-joint stiffness/damping/armature from the MJCF;
      obs/act 21→28 ripple validation for training; offline poselib retargeting → reference clips;
      reduced-backend parity. Reference assets: `~/Projects/research/humanoid-motion/ASE/ase/`.
- [ ] **Integrate with the training infrastructure (built elsewhere)** — connect the engine's
      env/config/gradient surfaces to the external trainer (reward/policy/optimizer, experiment configs,
      logging). Define the boundary: Python bindings + config `dump()` + obs/action layout + per-step
      Jacobian/VJP + the α-order hybrid. This is where the SHAC smoke test + PPO path actually run.
- [~] **Another round of testing** (post-config, pre-training) — unit/integration/benchmark/visual pass
      focused on the config system (override/serialize/hash edge cases), the `Environment`/`VecEnv` boundary,
      and end-to-end batched-rollout determinism/reproducibility before the training stack lands.
      **PARTIAL (2026-07-04):** AMP-rig-through-`physics_env::VecEnv` integration test added
      (`tst/physics_env/integration/amp_vecenv.cpp` — dims 28/84 on both backends, sound passive + gently-
      actuated rollout, parallel==serial bit-identical). Remaining: config override/serialize/hash edge cases
      + wiring `dump()` into a rollout and asserting run-to-run reproducibility.
- [x] **Graphics-free (headless) training build** — DONE (2026-07-04). `ENGINE_TRAINING_ONLY` CMake option
      builds ONLY `core + physics + physics_env` (skips graphics/RHI/GLFW/Metal/Vulkan/shaders/tests) + an
      `engine::training` INTERFACE aggregate for the binding to link. Verified: training-only configure+build
      on macOS produces just `libengine_{core,physics,physics_env}.a` (+ tinyobj), no graphics/GLFW — so the
      Python binding links clean on a GPU/display-less Linux box (no `.mm` in the training modules). sim-1
      `csrc` now links `engine::training` and configures the engine with `ENGINE_TRAINING_ONLY=ON`.
- [ ] **RSI (reference-state init) via the binding** — expose per-env arbitrary-state reset (not just the
      authored pose; incl. fallen states) through `Environment::setBodyState` + the reset hook. Not needed
      for PPO balance/walk (P1–P2); REQUIRED for tracking/imitation (SuperTrack-style, P3–P4).
- [ ] **Richer observation surface for tracking** — expose per-link root-local positions/velocities +
      6D rotations (the SuperTrack feature set) beyond the default packer; the engine has the raw data
      (`links()`/`jointStates()`), the binding just needs to surface it. Add when P3 (tracking) starts.
- [ ] **`DiffEnvironment` binding for SHAC** — wrap `jacobian()`/`rolloutGradient()` as a torch
      autograd Function (custom VJP) for the analytic policy gradient. Deliberately after the PPO baseline (P3).
- [x] **Deep-dive testing: semi-implicit contact + contact force model** (2026-07-04). Report:
      [2026-07-04-diff-semiimplicit-testing.md](../investigations/physics/2026-07-04-diff-semiimplicit-testing.md).
      Added `tst/physics/unit/diff_semiimplicit.cpp` (adhesion probe / smooth-dynamics damping / freefall
      equivalence / determinism), a humanoid explicit-vs-semi rest-parity integration test, and
      `ENGINE_SEMI`/`ENGINE_SUBSTEPS` toggles on the `diff_humanoid` visual. Tree green (141/0). Fixes
      deferred (below).
- [x] **FIX: contact normal force is adhesive during separation** — DONE (2026-07-04). Approach-gated
      damping `σ(−groundDampBeta·vn)` (new `DiffModel::groundDampBeta`) switches damping off on separation
      ⇒ `Fn ≥ 0` (non-adhesive), compression damping intact (low bounce). Measured Fn(vn=+2) −99.6 N→+68.4 N.
      Guard `diff_ground_force_non_adhesive`.
- [x] **FIX: semi-implicit damped the whole system → IMEX** — DONE (2026-07-04). SemiImplicit now treats
      ONLY the stiff contact force implicitly (contact @ predicted state via `computeContactForcesWorld` +
      `extContactWorld` arg on `diffForwardDynamics`), smooth dynamics stay explicit/symplectic. Contact-free
      pendulum E/E₀ 0.63→1.007 (= explicit); stiff-contact stability + gradients retained. Guard
      `diff_semiimplicit_imex_preserves_smooth_energy`.
- [x] **Whole-system damping = optional PHYSICAL knob (not numerical)** — DONE (2026-07-04). Added
      `DiffModel::jointDamping` (viscous `τ=−b·q̇`, default 0). Timestep-independent + differentiable
      (verified identical at h and h/2); the diff humanoid doesn't need it for stability, it's for realism /
      settling free DOFs / matching the RL backend (ties into cross-engine config unification). Guard
      `diff_joint_damping_physical`. Report: [2026-07-04-diff-semiimplicit-testing.md](../investigations/physics/2026-07-04-diff-semiimplicit-testing.md) (Resolution).
- [x] contact-solve perf: **sparse LDLᵀ factorization of `H`** exploiting the DOF-ancestor tree
      (replaces the dense O(ndof³) inverse). DONE (2026-07-04): ~1.5–1.65× env-steps/s for the
      reduced humanoid (N=1024: 16.5k→26.8k); validated vs the dense inverse (≤1.5e-4). When contacts
      are many, the **PGS** dominates instead — future lever: block/Delassus PGS.
- [x] contact-solve perf II: **warm-started PGS + 2×2 block friction (circular cone) + manifold
      reduction** (Delassus skipped — wrong tool when contacts > ndof). DONE (2026-07-04): 20→12
      iters at equal quality; flat-humanoid contact-solve −26% (0.989→0.732 ms), reduced env-steps/s
      +18–22%. End-to-end vs original dense-inverse ≈ 1.9× (N=1024 16.5k→31.3k). Analysis + results:
      [2026-07-04-reduced-contact-pgs.md](../investigations/physics/2026-07-04-reduced-contact-pgs.md).
- [x] reduced-backend coverage hardening (2026-07-04): added `tst/physics/integration/reduced_joints.cpp`
      (fixed/revolute-torque/revolute-PD/off-axis hinge/capsule) + visual gallery
      `tst/physics/visual/reduced_joints.cpp`. Surfaced + fixed two bugs: **joint limits** (were
      ignored → now impulse-based one-sided constraints in the generalized solver) and **WorldDef
      damping on the floating base** (linearDamping ignored → now damps `baseTwist_`). Stress env
      `substeps` 24→48 for the stiffer limited dynamics. See the reduced-coordinate design doc.
- [x] **Ball q/qd multi-DOF readout** (2026-07-04): `JointState` gained `rotation` (rest-relative
      orientation as a rotation vector) + `angularVelocity`; populated by both backends (reduced from
      `locRot`/`qd`, realtime from `refRel`/poses). Default obs packer is now **DOF-complete**
      (revolute→q,qd; ball→rotvec,ω), so humanoid `obsDim` 53→69.
- [x] **PD-target action mode** for the env (2026-07-04): `EnvConfig::actionMode {Torque,PDTarget}`
      + `kp`/`kd`; PDTarget interprets the action as a desired joint position (revolute angle / ball
      orientation-rotvec), tracked by the actuator PD servo. Added `setJointBallTarget` to the
      `PhysicsWorld` interface + both backends. Tests: `reduced_ball_state_readout`,
      `reduced_env_pd_target_tracks` (knee tracks a commanded bend).

## Core (mostly done — geometry/primitives/Handle/Transform/threading landed; image + io remain)

Build `engine::core` to a good state before touching graphics. See
[2026-07-02-core-module-plan.md](../investigations/core/2026-07-02-core-module-plan.md).

- [x] **geometry/**: `Vertex` (position, **normal**, uv, color), `MeshData`, `Mesh`,
      minimal `Material`, `ModelData`. + `core.h` umbrella. DONE.
- [x] **geometry/primitives**: `makeQuad` / `makePlane` / `makeSphere` generators. DONE
      (feed the milestone + give `tst` real geometry without asset files;
      `tst/core/unit/geometry.cpp`).
- [ ] **image/**: `Image` + `ImageFormat`.
- [ ] **io/**: `readFile`, `loadObj` → `ModelData`, `loadImage` → `Image`. NOTE: centralize
      the stb/tinyobj implementation macros in exactly one TU (they currently live in
      graphics) to avoid duplicate symbols when both link — do this when graphics is
      refactored, or keep loaders' impls out of any target that also links graphics.
- [x] **tst driver**: build meshes via primitives, assert counts, link `engine::core` only.
      DONE (`tst/core/unit/geometry.cpp`).
- [x] Also landed in `core` (not originally listed here): `memory/Handle<Tag>` (shared by rhi +
      ecs), `math/Transform`, and `threading/` (`ThreadPool` + `parallelSort`).

Scaling / ownership (from [2026-07-02-geometry-scaling.md](../investigations/core/2026-07-02-geometry-scaling.md)):
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
[2026-07-01-graphics-refactor.md](../investigations/realtime-rendering/2026-07-01-graphics-refactor.md) and
[2026-07-02-metal-backend.md](../investigations/realtime-rendering/2026-07-02-metal-backend.md).
**Key framing: "add Metal" and "do the refactor" are the same effort** — both require
extracting a backend-agnostic interface (RHI) and putting Vulkan behind it.

- [x] **0. Decide: build-your-own RHI vs. adopt one.** DECIDED (2026-07-02): **build our
      own** — no third-party RHI. Both Vulkan and Metal backends are hand-written.
- [ ] **1. Adopt `engine::core` in graphics**: replace graphics' `Vertex`/`Mesh`/`Material`
      and its OBJ/image loading with core's; delete the now-duplicated code. Each backend
      maps `core::Vertex` to its own vertex format. (Core itself is built in the Core section
      above, before this pass.)
- [x] **2. Define the RHI interface** — DONE (headers, 2026-07-02). Landed under
      `include/engine/graphics/rhi/` (types, resources, pipeline, command_list, device +
      `rhi.h` umbrella) and `include/engine/graphics/render/` (render_view, geometry_store,
      renderer). Interface-only, no backend, compiles clean (`-Wall -Wextra`). Decisions:
      handle-based, compile-time backend, **bindless**, Vulkan dynamic rendering; **Metal
      first**; **Slang** shaders. Design + sequencing:
      [2026-07-02-rhi-interface-plan.md](../investigations/realtime-rendering/2026-07-02-rhi-interface-plan.md)
      (§13 supersedes the ordering below).
- [ ] **3. Reorganize existing Vulkan code** into `src/graphics/vulkan/` behind the RHI;
      delete the 3 duplicated pipeline builders; fold `graphics_custom.cpp` offscreen helpers
      into the pipeline/pass API. (Design Vulkan side around dynamic rendering to line up
      with Metal.)
- [~] **4. Multi-backend CMake** — mostly DONE (2026-07-02): **Foundation** linked, `external/
      metal-cpp` on the include path, one TU (`metal/metal_backend.cpp`) defines the impl
      macros, **per-backend source selection** (`metal/` vs `vulkan/` + shared `common/`),
      `test`→`tst`, `ENGINE_RHI_METAL/VULKAN` defines. Remaining: enable `OBJCXX` when the
      `.mm` window shim lands (headless Metal is pure C++, so not needed yet).
- [x] **5. Shader toolchain** — DONE (2026-07-03): **Slang** chosen and wired.
      `src/tools/get_slang.sh` fetches `slangc` into gitignored `external/slang/`;
      `src/shaders/CMakeLists.txt` compiles `.slang` → `.metallib` (Apple) / `.spv` (else) and
      exposes `ENGINE_SHADER_DIR`. `rhi::ShaderModule`/`createShader` loads the backend blob.
      First shader: `src/shaders/triangle.slang`.
- [~] **6. Implement the Metal backend** incrementally — offscreen + **windowed** working
      (2026-07-03): `tst/graphics/integration/mesh.cpp` (headless, pixel-verified) and `tst/graphics/visual/grid.cpp`
      (opens a GLFW window, renders a lit `core` sphere via CAMetalLayer swapchain + present).
      Implemented: headless & windowed Device, handle pools, metallib libs, pipeline + vertex
      descriptor + depth-stencil, render-encoder lifecycle, indexed draw, depth, readback,
      CAMetalLayer/drawable present (`metal_window.mm` shim via `glfwGetCocoaWindow`;
      `OBJC`+`OBJCXX` enabled). **MVP camera uniform + instance storage + per-instance materials
      DONE** (see item 8 + the architecture "Build reality" callout). TODO: window resize
      (swapchain + depth recreate; currently fixed-size), staging for Private resources,
      fence-gated deletion, per-frame-in-flight sync, and **bindless textures** (stubs exist).
- [ ] **7. Split `Swapchain` + `Renderer`; headless path** for both backends (ML/offline).
- [~] **8. Rework the render-list / instance path** — instancing + **per-instance materials**
      done (2026-07-03): `RenderView { view/proj, target, RenderItem[], InstanceData[],
      MaterialGPU[] }` consumed by `render::Renderer`; per-instance SoA data + a materials
      storage buffer (each instance's `materialIndex` → `baseColorFactor`); one instanced
      `drawIndexed` per RenderItem. Verified: `mesh_offscreen` (3 differently-colored instanced
      spheres, red center), `visual_window` (NxN color-gradient grid, `ENGINE_GRID=N`).
      Remaining: **bindless texture table** (`baseColorTexture` + `registerBindlessTexture` are
      stubs — defer until textured surfaces are needed; Metal argument-buffer + residency work);
      ECS-driven extraction/culling/sorting once ECS exists; the indirect/GPU-driven path.

Known bugs/smells to fix along the way (all in the **parked legacy Vulkan code** under
`src/graphics/vulkan/` — fix during the Vulkan-behind-RHI port, not present in the Metal path;
details in the refactor investigation):
- [ ] `loadQuad` mis-computes `vertexCount`/`indexCount` (cumulative, not per-mesh counts).
- [ ] `copyInstanceToBuffer` always copies `MAX_ENTITIES`; dirty-range copy is stubbed out.
- [ ] `renderFinishedSemaphores` indexed per-frame not per-image (masked by frames=1).
- [ ] macOS `vkQueueWaitIdle` after every present + `MAX_FRAMES_IN_FLIGHT = 1` → no
      pipelining. Revisit with multiple frames in flight.
- [ ] "paint"/"stamp" naming leftovers from the source app.
- [ ] `GlobalUBO` lights (`// todo: lights`).
- [ ] Deferred deletion queue (readme.md sketch), once resources outlive a frame.

## Render framework — clustered forward+ render graph (ACTIVE, 2026-07-04)

The mid-level framework between ECS extraction and the RHI. Everything modern (multi-light,
shadows, AA, sky, post) depends on it. Design + diagram + perf/backend/multithreading review:
[2026-07-04-render-framework-plan.md](../investigations/realtime-rendering/2026-07-04-render-framework-plan.md).
**DECIDED (owner, 2026-07-04)**: lighting = **clustered Forward+** (graph kept G-buffer-capable
for a later deferred/offline path); render graph is **explicit + correctness-first** (no memory
aliasing / pass-culling in v1). Grass + ray tracing + full deferred deferred by owner.

**Head-swap prep DONE (2026-07-06)** — groundwork so the graphics head is swappable (realtime |
path tracer) before the path tracer is written. (a) De-rasterized the `RenderView` contract:
`RenderItem = {mesh, firstInstance, instanceCount}` (no pipeline); the realtime renderer holds the
opaque mesh pipeline (`RenderResources.mesh` / `Renderer::setMeshPipeline`, bound once);
`scene::extract(world, out)` is pipeline-free. (b) Moved the neutral contract to
`include/engine/graphics/view/render_view.h`. (c) Split `engine::graphics` → **`engine::rhi`** (rhi/
+ view/ + backend, head-agnostic base) + **`engine::render`** (realtime renderer); `scene` now
depends only on `engine::rhi`. (d) Reorganized graphics tests into `tst/graphics/realtime/{unit,
integration,benchmark,visual}` with `tst/graphics/path-tracer/` ready. Behavior-preserving: suite
167/0, benchmark MSAA delta nominal. A path tracer is now a clean `engine::pathtracer` sibling over
`engine::rhi` consuming the same `RenderView`. Notes:
[head-swap-readiness](../investigations/path-tracing/2026-07-06-renderer-head-swap-readiness.md),
[refactor-plan](../investigations/path-tracing/2026-07-06-head-swap-refactor-plan.md).

**Path tracer — step 1 DONE (2026-07-06)** — a glm-ported CPU **reference** path tracer as a new
`engine::pathtracer` module (CPU + glm + `engine::core` only; **no Eigen, no RHI yet**). Salvaged
the integrator IP from the old `path-hypochoco` student tracer, dropping its Qt/Eigen/CS123/BVH
infrastructure (assessment:
[salvage note](../investigations/path-tracing/2026-07-06-path-tracer-salvage-assessment.md)).
`pt::Scene` (world-space triangle soup + materials + emissive index + pinhole camera, fed from
`core::MeshData`), `pt::render()` (area-light NEE + cosine-weighted indirect + Fresnel dielectric +
mirror + Russian roulette), `pt::toneMap()` (Reinhard+gamma). **Bugs fixed** vs the original:
correct area-measure direct-lighting estimator, uniform (√r1) triangle sampling, cosine-weighted
hemisphere sampling, epsilon-offset ray origins, per-pixel deterministic RNG. **Brute-force
intersection (BVH deferred** — engine has no ray BVH; physics has only broadphase). Pure primitives
(intersection, sampling, Fresnel, basis) extracted to header `engine/pathtracer/sampling.h`
(reusable for the GPU/BVH paths + unit-testable); `Scene::intersect`/`occluded` exposed. **Tests
(10, module `pathtracer`)**: unit — `ray_triangle`, `scene_nearest_and_occlusion`, `fresnel_schlick`,
`orthonormal_basis`, `cosine_hemisphere` (MC ∫cos²θ dω = 2π/3), `triangle_sampling` (mean→centroid);
integration — `cornell_box`, `no_emitter_is_black`, `emitter_triangulation_invariance` (2 vs 4-tri
light identical, rel=0.000 — guards the estimator fix), `mirror_reflects_light`. Suite 177/0.
(Forward plan refined by the dependency review — see step 1.5 + the roadmap below.)

**Path tracer — step 1.5 (geometry catalog + side-by-side) DONE (2026-07-07)** — dependency review
([note](../investigations/path-tracing/2026-07-07-pathtracer-dependency-model.md)) settled: keep
`engine::pathtracer` **core-only**; each head gets an ECS bridge (`pathtracer : pathtracer_scene ::
physics : physics_ecs`; `render`'s bridge is the existing `engine::scene`). Refactor landed: a
**`engine::core` `GeometryCatalog`** (`core::MeshId` → `MeshData`) — the neutral, renderer-agnostic
CPU geometry residency both heads (and the future BVH) source from. Side-by-side comparison
`pathtracer.raster_vs_pathtraced`: one sphere registered once in the catalog, rendered by the raster
head (GeometryStore + Renderer, offscreen→readback) AND the path tracer (`pt::Scene` from the same
catalog mesh) at the same camera; writes `raster_vs_pathtraced.png` + asserts the raster silhouette
matches the PT intersector's geometry (IoU=1.000). Unit test `core.geometry_catalog`. Suite 179/0.
**Deferred**: the `pathtracer_scene` ECS→`pt::Scene` bridge + ECS `RenderMesh`→`MeshId` rewire +
`scene`→`render_scene` rename; a live windowed A/B (needs texture-blit plumbing). **Next**: the ECS
bridge, then the Slang-compute GPU path, then a data-oriented BVH (built over the catalog).

**Path tracer — roadmap (paused 2026-07-08; resume here).** Status: **CPU reference tracer is
correct, tested (11 pathtracer + 1 core catalog tests), and demoable** side-by-side vs the raster
head. `engine::pathtracer` is core-only; geometry is neutral in `core::GeometryCatalog`. Remaining
work, in recommended order:
  1. **`engine::pathtracer_scene` ECS bridge** (`ecs + pathtracer + core`) — extract an ECS world into
     a `pt::Scene`, mirroring `engine::scene`→`RenderView`. Requires rewiring ECS `RenderMesh` to hold
     a `core::MeshId` (from the catalog) instead of a `render::MeshHandle`, and having `scene::extract`
     resolve `MeshId`→GPU. Completes the head-swap symmetry (path-trace the *authoritative* ECS scene).
  2. **Material extension** — add `emission` (+ a BSDF `type`/flags) to the shared material so PT gets
     lights/specular from the same material both heads read (kept one shared model for now; subclass
     into realtime/PT materials later if they diverge).
  3. **GPU compute path** — port the integrator to a Slang compute shader over the RHI (geometry in
     GPU buffers, per-pixel RNG, an accumulation buffer; megakernel first). The real perf lift;
     regression-test GPU output against the CPU reference on the same scene.
  4. **Data-oriented BVH** over the `GeometryCatalog` (possibly sharing the physics `aabb` primitive) —
     once brute-force intersection is the bottleneck. CPU first (validate), then GPU-traversable.
  5. **Live windowed A/B demo** — needs texture-upload + a blit/textured-quad pass to composite the
     realtime view and the (uploaded) path-traced texture into one window.
Design notes: [salvage](../investigations/path-tracing/2026-07-06-path-tracer-salvage-assessment.md),
[dependency model](../investigations/path-tracing/2026-07-07-pathtracer-dependency-model.md),
[head-swap readiness](../investigations/path-tracing/2026-07-06-renderer-head-swap-readiness.md).

- [x] **RF1. RHI additions** — DONE (2026-07-04). `transient` texture hint → Metal
      `MTLStorageModeMemoryless` for render-target-only textures (renderer depth is now transient);
      `CommandList::resourceBarrier(span<ResourceTransition>)` + `ResourceState` enum (Vulkan real
      barriers later; Metal near-no-op, auto-tracked). Test `graphics.barrier_transient` (transient
      depth + barriers, clear read back 64/128/191/255). Parity anchors intact.
- [x] **RF2. FrameRingAllocator + per-view sub-allocation** — DONE (2026-07-04).
      `include/engine/graphics/render/frame_ring.h` (header-only: per-frame-in-flight arenas +
      per-view bump sub-alloc, grow-by-retire). Metal backend now **cycles frameIndex** +
      `dispatch_semaphore(framesInFlight)` throttle (completion-handler signal) — fixes the real
      windowed hazard by construction; `Device::framesInFlight()` added. Renderer reworked to
      ring-allocate camera/instances/materials per view. Test `graphics.multiview_ring` (2 views/
      frame → 2 targets over 3 frames; A red r153/b24, B blue r23/b157 — proves no clobber).
- [x] **RF3. Minimal RenderGraph** — DONE (2026-07-04).
      `include/engine/graphics/render/render_graph.h` (header-only; addRasterPass/addComputePass +
      declared reads/writes; registration-order execute with auto `resourceBarrier` + begin/end
      Rendering wrapping the record callback; state tracked per texture). Renderer builds a graph
      (one forward raster pass per view) — `render()` API unchanged so mesh/scene/triangle/multiview
      are the pixel-parity check (all hold). Reorder/aliasing/pass-culling still deferred (per scope).
- [~] **RF4a. Multi-light forward (loop-all)** — DONE (2026-07-04). `render::PointLight` +
      `RenderView.pointLights`; `GlobalUniforms.params.x` = light count; ring-uploaded + bound at
      buffer 3; `mesh.slang` adds world-pos + a punctual light loop (Lambert + smooth range
      attenuation). Test `graphics.multi_light` (ambient-only center red=13 → 1 point light=255 →
      4 lights=255). This is the correct data path; clustering (RF4b) is the perf scaling on top.
- [x] **RF4b. Cluster-binning compute pass (clustered forward+)** — DONE + verified (2026-07-04).
      **Metal compute subsystem**: `createComputePipeline` (`MTL::ComputePipelineState`) + a compute
      encoder path (`CommandList::beginCompute/endCompute`, `bindPipeline`/`bindResources`/`dispatch`
      compute-aware) — smoke test `graphics.compute_smoke` (kernel writes i·2+1 → readback). **Binning**:
      `src/shaders/cluster.slang` (one thread/froxel: view-space AABB + sphere-overlap test → per-cluster
      light index list); `graphics.cluster_binning` verifies GPU==CPU reference over all 512 clusters
      (0 mismatches) + in-frustum light binned / out-of-frustum lights culled. **Integration**:
      `mesh.slang` clustered path (froxel lookup from SV_Position + view-space depth) behind a param
      flag; `Renderer::setClusterBinning(pipeline)` adds a per-view binning compute pass before the
      forward pass. `graphics.clustered_forward` proves clustered output is **pixel-identical** to
      loop-all (MAD=0.0, maxDiff=0 over 22.8k lit px). Grid 12×12×24, maxLights/cluster 64.
      **Note**: an early benchmark showed clustered ≈ loop-all; that was NOT a binning-cost issue
      but two bugs (uncapped forward loop + the `params.y` "use clusters" flag set AFTER the camera
      upload, so the clustered branch never ran — the equivalence test couldn't catch it as both
      paths were loop-all). Both fixed in RF4b-opt, which then measured up to **17.9×**.
      Deferred (still): full LightList (spot lights), Z-slice exponential distribution.
- [x] **RF4b-opt. Faster light binning** — DONE (2026-07-04). Rewrote `cluster.slang` as a
      **thread-per-light scatter**: each light computes the small froxel range it covers (tile
      range + depth-slice range, +1 conservative margin) and appends itself into overlapping
      clusters via `InterlockedAdd` on the per-cluster count (host zero-clears the count buffer
      each frame; dispatch is now per-light). O(lights × local-froxels) vs the old
      O(clusters × lights). **Two bugs found + fixed while benchmarking** (this is why the earlier
      "naive binning offsets the win" note was wrong — clustering wasn't actually running):
      (1) the forward shader looped the **uncapped** atomic count and read past the 64-entry list
      into garbage — now `min(count, maxPer)`; (2) the renderer set `params.y` (the "use clusters"
      flag) **after** uploading the camera uniform, so the shader always took the loop-all branch —
      the `clustered_forward` equivalence test couldn't catch it (both paths were loop-all). Fixed
      the ordering; clustering now genuinely runs. **Result** (wide field of 4096 spheres, local
      lights, 512×512): clustered vs loop-all **256 lights 2.3×** (1.83→0.81 ms), **1024 5.3×**
      (4.90→0.93 ms), **4096 17.9×** (17.7→0.99 ms) — clustered stays ~1 ms as lights scale.
      Correctness held throughout (`clustered_forward` MAD=0, `cluster_binning` 0 mismatches).
- [x] **RF5. HDR + tonemap** — DONE (2026-07-04). Added **texture+sampler binding** to the RHI
      (`TextureBinding`/`SamplerBinding` in `ResourceBindings`; Metal `createSampler` + fragment
      texture/sampler binds — the first shader texture sampling). `tonemap.slang` = fullscreen
      triangle (SV_VertexID) + ACES (Narkowicz) + sRGB. `Renderer::setTonemap(pipeline, sampler)`
      opt-in: forward renders into an RGBA16F HDR target, then a fullscreen tonemap pass resolves to
      view.target (the graph inserts the HDR RenderTarget→ShaderRead barrier). Test
      `graphics.hdr_tonemap` (bright ambient: tonemap OFF clips to 255, ON=241 via ACES). Wired into
      the `clustered_lights` visual. App builds the mesh pipeline as RGBA16Float + a tonemap pipeline
      (cull None, no depth) matching the final target when tonemapping.
- [~] **RF6. Shadows / sky / AA** (additive graph nodes). **Shadows DONE (2026-07-04)**:
      directional sun shadow map — a depth-only shadow pass (`shadow.slang`, needs the new
      null-fragment/depth-only pipeline + a sampleable depth texture + a color-less graph pass)
      renders the scene from the sun's ortho view into a 2048² depth map; the forward shader
      PCF-samples it (3×3) to shadow the directional term. `Renderer::setShadows(pipeline, sampler,
      extent, dist)` opt-in; the graph barriers the map RenderTarget→ShaderRead. Test
      `graphics.shadow_map` (caster over ground → localized shadow patch: 1020 px darken, ground
      outside stays lit); visual `tst/graphics/visual/shadow_scene.cpp` (cubes + rotating sun + HDR).
      **Known artifact — peter-panning** (verified `graphics.shadow_bias`, 2026-07-05): a constant
      depth bias detaches the shadow slightly from a caster's base. **Fixed with slope-scaled bias**
      (2026-07-05): the shader scales bias by the surface's angle to the sun — near the floor for
      surfaces facing the light (reconnects contact shadows) rising to the max at grazing (prevents
      acne); exposed via `setShadows(..., bias)` where the value is the max (negative = legacy
      constant, for A/B tests), passed through `lightDir.w`. Test `graphics.shadow_bias` confirms the
      adaptive behavior at the clean end: on a grazing ground a low constant bias self-shadows
      (5599 acne px) while slope-scaled → 0. **Honest caveat**: in *steep-sun* configs the shadow
      boundary is a depth cliff largely insensitive to bias in the usable range, so slope-scaling's
      reduction of the base gap there is modest; for a hard contact fix on solid geometry, **front-
      face culling in the shadow pass** is stronger (blocked on a separate finding: `RasterState.cull`
      is currently not applied in the Metal backend — culling defaults to none).
      **Next**: AA (MSAA or post). Later: cascaded shadow maps (CSM) for large outdoor range,
      PCF→PCSS soft edges.
      **Sky/atmosphere DONE (2026-07-05)**: procedural sun-coupled sky (`sky.slang`) — cheap
      analytic horizon→zenith gradient warmed toward the sun + HDR sun disc + forward-scatter glow
      (~20 ALU/pixel, no LUTs; design note `investigations/realtime-rendering/2026-07-05-sky-atmosphere.md`). Drawn as
      a far-plane fullscreen triangle at the END of the forward pass (same render pass ⇒ on-tile on
      Apple, no HDR store/load; the depth buffer is transient/memoryless so a separate pass couldn't
      read it anyway), depth-tested LessEqual + no depth write ⇒ fills only background pixels.
      `Renderer::setSky(pipeline)` + `setSkyColors(...)` opt-in, coupled to the view sun. Test
      `graphics.sky` (opaque untouched by the sky = depth test; zenith blue-bias 56 vs horizon 15;
      sun-front bg max 590 vs sun-behind 283). Wired into the `shadow_scene` visual. **Benchmark
      before/after** (`render_graph`, 512², Apple): sky overhead **~0.011–0.012 ms/frame** (fullscreen
      background fill — negligible), matching the perf-lean design. Deferred: Hillaire multi-scatter
      LUT sky upgrade, sky-as-IBL for ambient.
      **Atmosphere (aerial perspective + height fog) DONE (2026-07-05)**: distant/low opaque
      fragments blend toward a sky-consistent fog color + Mie-like sun in-scatter, applied as the
      last step of `mesh.slang` (fog on geometry needs per-fragment world pos + lit color; depth is
      memoryless so no separate pass). Exponential distance × height falloff.
      `Renderer::setFog(density, heightFalloff, baseHeight, color, inscatterColor, inscatterExp)`
      opt-in, **off by default** (parity anchors intact). Design note
      `investigations/realtime-rendering/2026-07-05-atmosphere-aerial-perspective.md`. Test `graphics.fog` (far object
      washes toward fog color + monotonic with distance; sun-ahead in-scatter 765 vs 486 behind;
      height fog base foggier than top). Wired into `shadow_scene`. **Benchmark ~0.005 ms** (few
      fragment ALU, within noise). Deferred: Hillaire aerial-perspective 3D LUT, volumetric fog /
      light shafts, fog influencing the sky pass.
      **Known limitation (2026-07-05)**: fog blends geometry toward a *constant* `fogColor`, but the
      sky pass is *not* fogged, so distant geometry (→fogColor) meets the sky (→sky-horizon color) at
      a visible seam on the horizon line. Cheap mitigation = set `fogColor` ≈ the sky horizon color.
      Proper fix (the correct "aerial perspective") = make the fog color **view-direction dependent**:
      fade geometry toward the *sky color along that ray* (evaluate the sky gradient in `mesh.slang`
      from the sky palette passed in Globals) so geometry converges exactly to the sky behind it —
      no seam. Deferred pending owner call (flagged 2026-07-05; demo look is acceptable as-is).
      **AA (MSAA + FXAA) DONE (2026-07-05)** — RF6 shadows/sky/AA trio complete. Hardware MSAA on
      the forward pass: RHI honors `TextureDesc.sampleCount` (Metal `Type2DMultisample`),
      `createGraphicsPipeline` sets `rasterSampleCount`, `ColorAttachment.resolveTarget` →
      `StoreActionMultisampleResolve`; `Renderer::setMSAA(n)` renders memoryless MSAA color+depth and
      resolves on-tile into the single-sample HDR/view target (app builds mesh+sky pipelines with
      matching sampleCount). Optional FXAA post pass (`fxaa.slang`, compact luma-FXAA) via
      `Renderer::setFXAA` with tonemap→intermediate-LDR→FXAA→present ordering (catches the sun-disc
      shading aliasing MSAA misses). Both opt-in, off by default. Design note
      `investigations/realtime-rendering/2026-07-05-antialiasing-msaa-fxaa.md`. Test `graphics.aa` (rotated slab: 0
      partial-coverage edge px → 540 @4× MSAA, 628 @FXAA; interior/bg untouched). Benchmark: MSAA 4×
      **+0.29/+1.2 ms** @4k/16k instances (4× coverage raster, on-tile resolve); FXAA **~0.01 ms**.
      Wired into `atmosphere_scene` (M/X toggles). Deferred: TAA/MetalFX temporal, SMAA, custom
      tonemap-weighted HDR MSAA resolve, alpha-to-coverage.
      **Graphics config system DONE (2026-07-05)** — the scattered `Renderer::setX()` toggles +
      tuning knobs are centralized into a value-type `GraphicsConfig` (nested Shadow/Sky/Fog/AA/
      Cluster + hdr) split from a `RenderResources` handle bundle (feature active = config.enabled &&
      resource valid). `Renderer::setConfig`/`setResources`; the 7 setters are thin wrappers.
      Un-buried `shadow::MAP_SIZE`→`shadow.mapSize` (now tunable) + the froxel grid→`ClusterConfig`
      (compile-time-effective, decision 5). G2: `GraphicsConfigOverride` + `resolve(base, override)`
      + `presets::{baseline,performance,cinematic}`. G3: write-only `serialize`/`configHash`/`dump`
      (`graphics_config_io.h`; reader deferred). Tests `graphics.config_*` (4 cases); `atmosphere_scene`
      migrated to drive its toggles from one config. Suite 167/0 (was 163 + 4 config). Design note
      `investigations/realtime-rendering/2026-07-05-graphics-config-system.md`. Deferred: kv READER (needs a tools/CLI),
      runtime froxel-grid resize, a pipeline-variant helper to remove the app's MSAA sample-count juggling.
- [x] **Benchmark** — DONE (2026-07-04), `tst/graphics/benchmark/render_graph.cpp` (in the
      `benchmarks` runner; graphics bench now globbed + `engine::graphics` linked). Numbers (Apple,
      RelWithDebInfo, 512×512, headless — relative baseline for THIS machine):
      **instance sweep (0 lights)** CPU record 0.011/0.022/0.056/0.214 ms and frame(incl GPU)
      0.66/2.0/3.1/10.5 ms at 256/4k/16k/65k instances → the graph+ring CPU path stays flat/cheap.
      **light sweep (4096 inst)** frame 1.06/1.21/2.33/7.15 ms at 0/16/64/256 lights while CPU record
      stays ~0.02 ms → confirms the cost is GPU fragment shading looping all lights ⇒ **clustering
      (RF4b) is the right lever** (scale with lights-per-cluster, not total lights).
- **Multithreading (notes only, not now)**: parallel command recording (per-pass, once record time
      is a measured bottleneck), parallel extraction (`parallelFor` per archetype chunk), and
      **parallel-world rendering** (mirrors the physics parallel-worlds win) — the graph is per-view
      so these slot in without contract changes. Do NOT thread the Device pools / submit. See the
      note §9.

## Compute / ML

- [ ] **Compute pipeline + compute-queue support.** Needed for ML workloads and some
      rendering; only graphics pipelines exist today. (Not needed for CPU training — the sim is CPU-bound.)
- [x] **Parallel environment stepping / batching** for training throughput. DONE via Milestone 2 **Phase D**
      (`Environment`/`VecEnv` over parallel `PhysicsWorld`s + SoA obs/action tensors; ~8.7k→68.7k env-steps/s).
- [x] **Determinism review** for reproducible training. DONE via **Phase D** (end-to-end batched-rollout
      determinism, parallel==serial bit-identical) + the AMP-rig VecEnv determinism test.

## Physics

Plan (all 8 decisions settled): [2026-07-03-physics-plan.md](../investigations/physics/2026-07-03-physics-plan.md).
Multi-backend behind a **runtime-virtual** `PhysicsWorld`; shared collision/broadphase
substrate; realtime (impulse) + implicit/**differentiable** backends; rotational dynamics.

- [x] **Simulation core.** **Phase 0 + Phase 1 DONE** (2026-07-03): ECS-free `engine::physics`
      core (shapes, exact contacts, rigid-body state + inertia, pure integration kernels incl.
      SO(3) exp/log) + the runtime-virtual **`PhysicsWorld` interface** + a **realtime
      sequential-impulse backend** (restitution/friction/Baumgarte, substeps, rotation) + the
      **`engine::physics_ecs` bridge** (RigidBody + step/sync systems). **Milestone met**:
      `tst/physics/integration/milestone.cpp` (headless — ball rolls without slipping down a 30°
      incline, analytic match) + `tst/physics/visual/rolling.cpp` (windowed, full stack).
      **Phase 2 + collision polish COMPLETE**:
      sweep-and-prune + **uniform-grid** (flat index-sort) broadphases, both verified vs brute
      force. Grid **scales linearly** — at 65,536 bodies grid 19.9 ms/step vs SAP 192 ms vs
      ~14 s for the old O(n²) (~700×); 100k ≈ 30 ms/step single-threaded. **Parallel worlds**
      via `core::ThreadPool`: 7.8× on 12 workers (36.8M body-steps/s). **Colliders**:
      sphere/plane/box/hull/capsule; analytic sphere-box + capsule-sphere/capsule/plane;
      **GJK closest-distance** for capsule-vs-box/hull; GJK/EPA for the normal; **face-clip
      manifolds** (`box_box` + `convex_manifold`) → box/hull **stacking**. **Parallel broadphase
      sort**; **clamped Baumgarte**; **CCD** (swept AABBs + speculative contacts — 120 m/s
      sphere doesn't tunnel); **kinematic bodies move** by scripted velocity (ignore gravity/
      impulses); **restitution works under CCD** (speculative branch targets the rebound velocity
      instead of braking the approach — see
      [2026-07-03-physics-test-findings.md](../investigations/physics/2026-07-03-physics-test-findings.md)).
      Phase 2 + collision polish complete; tests live under `tst/physics/{unit,integration,
      benchmark,visual}/`. **Phase 3 DONE** (2026-07-04): reduced-coordinate (Phase E) +
      differentiable (Phase F) backends + the parallel-world ML harness (Phase D `VecEnv`).
- [x] **How physics state maps onto ECS components.** DECIDED (2026-07-03): backend owns
      packed state; ECS holds `RigidBody{BodyHandle}` (no pose) + keeps `Transform` separate
      (no replacement/inheritance); a `physics_ecs` bridge syncs world poses → Transform. ML
      path omits Transform entirely. (physics-plan Q2/Q3)
- [x] **Articulated bodies / joints + actuators** — DONE (2026-07-04) via Milestone-2 **Phase B**
      (maximal-coord joints/limits/actuators + humanoid preset), **Phase E** (reduced-coord Featherstone
      backend), and **Phase F** (differentiable). Both `makeHumanoid` (21 DOF) and `makeAMPHumanoid`
      (28 DOF) rigs. See the Phase B/E/F entries above.
- [ ] **Terrain collision** (Milestone 2, Phase C) — ⏸ **DEFERRED** (2026-07-03): flat `Plane`
      suffices for the humanoid for now. Heightfield collider + narrowphase; general concave
      triangle-mesh collider (decomposition or per-triangle+BVH+edge-filtering) is a separate
      larger item — note that **convex** meshes already work via `ConvexHull` (GJK/EPA).
      Design + analysis: [2026-07-03-terrain-collision-deferred.md](../investigations/physics/2026-07-03-terrain-collision-deferred.md).

## Infra / quality

- [~] Multithreaded task system. **`core::ThreadPool` landed** (fixed pool + blocking
      dynamic-work-stealing `parallelFor`, caller participates; `tst/core/unit/thread_pool.cpp`).
      Uses: **parallel worlds** 7.7× on 12 workers, and **intra-world** parallel integration +
      narrowphase + **graph-colored contact solver** (deterministic, bit-identical to serial;
      1.66× on a dense 32k pile) + **parallel broadphase sort** (`core::parallelSort`, landed).
      Still TODO: a task **dependency graph** (readme sketch), fewer per-color barriers.
- [x] Test setup. DONE (2026-07-03): self-registering harness + `tests`/`benchmarks`/`visuals`
      runners + CTest per module (see the Foundational "Driver test harness" item).

## Open design questions (revisit "at some point")

Parked decisions we agreed to defer. Answer before the work they gate.

- [x] **ECS approach: archetype vs sparse-set.** DECIDED (2026-07-03): **archetype** — best
      multi-component iteration/cache, maps onto batched instancing + the 100k case; structural
      changes (rare in a sim) are its weak spot but acceptable. (ecs-plan §3)
- [ ] **ML training model: many envs in-process vs separate processes.** Drives the
      headless context design and how far determinism must reach. (Lean: in-process vectorized
      via `ThreadPool` first — Milestone 2 Phase D; multi-process/distributed is downstream/cloud.)
- [~] **Articulation approach (Milestone 2): maximal-coordinate joint constraints vs
      reduced-coordinate (Featherstone/ABA).** DECIDED (2026-07-03):
      [2026-07-03-articulation-approach.md](../investigations/physics/2026-07-03-articulation-approach.md).
      **Constraints first** (Phase B — fast, reuses our solver, demonstrates the humanoid);
      **reduced-coordinate deferred but integral once we train** — it minimizes observation size
      (joint `q/qd` vs redundant body poses) → smaller policy inputs → higher throughput (a key
      training bottleneck), plus drift-free stability + cleaner gradients. Added later as a second
      `PhysicsWorld` backend; keep the joint/actuator + obs/action API backend-agnostic.
- [ ] **Terrain representation: heightfield vs general triangle-mesh collider.** Heightfield
      recommended (matches locomotion RL); tri-mesh possibly later.
- [ ] **Offline renderer basis:** is `graphics_custom.cpp`'s offscreen helper set intended
      to grow into the offline/high-quality renderer?
- [x] **Build-your-own RHI vs. adopt.** DECIDED (2026-07-02): build our own; no third-party
      RHI. (metal-backend §3)
- [x] **Shader language/toolchain**: DECIDED (2026-07-03) — **Slang** (one source → SPIR-V +
      metallib), wired via `slangc`. (metal-backend §5)
- [ ] **RHI dispatch**: runtime-virtual (simpler) vs. compile-time (zero overhead).
- [ ] **Metal API level**: classic `MTL` (widest support) vs. `MTL4` (newer, Vulkan-like;
      present in vendored metal-cpp).
- [ ] **Future D3D12 / Windows?** If likely, argues harder for adopting an RHI now.
