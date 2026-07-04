# Architecture (as-is)

Snapshot of the current structure. This describes what exists today, not the target.
Last synced with code: 2026-07-03.

## Build & layout

- C++23, CMake `>= 3.25`, `compile_commands.json` exported.
- **Default build type = RelWithDebInfo** (optimized) when none is given — a plain
  `cmake -S . -B build` would otherwise be unoptimized (no `-O`/`NDEBUG`), which runs physics
  ~20-50× slower and makes interactive visuals stutter/slow-mo. Override with
  `-DCMAKE_BUILD_TYPE=Debug`. (Multi-config generators pick per-build.)
- **Headless training build:** `-DENGINE_TRAINING_ONLY=ON` builds only the ECS-free sim surface
  (`core + physics + physics_env`) and skips the graphics stack (RHI/Metal/Vulkan), GLFW, shaders,
  and the test/visual runners — so the Python binding (`engine::training` aggregate) links clean on a
  GPU/display-less Linux box. Default (`OFF`) is the full dev build (`engine::engine`).
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
│   ├── physics_ecs/CMakeLists.txt  engine_physics_ecs(STATIC); physics↔ecs bridge
│   └── physics_env/CMakeLists.txt  engine_physics_env(STATIC); ECS-free RL env layer
├── src/shaders/                # .slang → .metallib/.spv via slangc (own CMake target)
├── src/tools/                  # build/dev tooling scripts (e.g. get_slang.sh)
├── tst/                        # tests by <module>/<category>/ → tests | benchmarks | visuals
└── external/                   # submodules: glfw, glm, stb, tinyobjloader, metal-cpp
```

## Target graph

```
engine::engine (INTERFACE)
├── engine::core        (STATIC)    → glm, tinyobjloader, stb, Threads
│                                     geometry, memory/Handle, math/{Transform,Camera}, Time, threading
├── engine::ecs         (STATIC)    → engine::core   (archetype World, queries, resources, Schedule)
├── engine::input       (INTERFACE) → engine::core, glm            (InputState + Key/MouseButton; GLFW-free)
├── engine::input_glfw  (STATIC)    → engine::input, glfw          (GlfwInput adapter; future: input_qt)
├── engine::controls    (STATIC)    → engine::ecs, engine::input, engine::core   (FlyController + system → Transform)
├── engine::graphics    (STATIC)    → engine::core, glfw, glm, tinyobjloader, stb,
│                                  (Apple) Metal+QuartzCore | (else) Vulkan::Vulkan   (RHI + Metal backend)
├── engine::scene       (STATIC) → engine::ecs + engine::graphics   (ECS↔render bridge; extract)
├── engine::physics     (STATIC) → engine::core   (shapes/collision/broadphase/solver, PhysicsWorld)
└── engine::physics_ecs (STATIC) → engine::physics + engine::ecs    (RigidBody + step/sync systems)
└── engine::physics_env (STATIC) → engine::physics + engine::core   (Environment + VecEnv; ECS-free RL layer)
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
> exist but aren't wired). Shader `src/shaders/mesh.slang` is instanced (SV_InstanceID → per-
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
shaders (`src/shaders/*.slang` → `.metallib`). This is what the tests and the milestone exercise.

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
- **Reduced backend** (`backends/reduced`, `Backend::Reduced`, Phase E, 2026-07-04): a second
  `PhysicsWorld` implementing the *same* interface via the **Articulated-Body Algorithm (ABA)** in
  **generalized coordinates** (joint q/qd + optional floating 6-DOF base) — O(n) over the limb
  tree, drift-free joints, minimal non-redundant state (what real humanoid-RL uses). The factory
  (`createPhysicsWorld`) dispatches on `Backend`; `createFeatherstoneWorld` is declared in the
  internal `backends/backends_internal.h`. Angular-first 6-vectors, full 6×6 spatial
  transforms/inertias (link frame at COM), gravity as an explicit per-link force, semi-implicit
  Euler + quaternion base integration. **Scope E0-E2**: **Fixed/Revolute/Ball** rotation joints
  (unified `relRot = restRel·locRot`, per-axis motion subspace), fixed + floating base, actuators
  (torque/PD, spherical PD for ball), and **contacts** — CRBA joint-space inertia `H` + generalized
  contact Jacobians + a sequential-impulse **PGS in generalized coordinates** (normal + Coulomb
  friction + Baumgarte) against static planes. Validated (`tst/physics/integration/reduced.cpp`):
  pendulum period 0.5%, double-pendulum + ball-pendulum energy ≤0.5%, floating free-chain momentum
  0.1%/0.24%, sphere rests / box holds+slides by friction, and the full **`makeHumanoid`** (14
  bodies/13 joints, floating pelvis) ragdoll-settles + holds a PD pose. `physics_env` can select it
  via `EnvConfig.backend` with no env-layer changes (E3 wires it into `VecEnv`). Roadmap:
  [reduced-coordinate-backend.md](../investigations/2026-07-04-reduced-coordinate-backend.md).
- **Joints** (Milestone 2, Phase B1, 2026-07-03): maximal-coordinate **bilateral constraints**
  solved in the same velocity loop as contacts, but **persistent** (created once via
  `createJoint(JointDef)`; kept in an index-stable `joints_` store) and **warm-started across
  steps**. Types: **Ball** (point-to-point, 3×3 K-matrix), **Revolute/hinge** (point + 2 angular
  axes), **Fixed/weld** (point + 3-axis orientation lock to a creation-time reference). Solved
  **serially in creation order** (deterministic) — angular part before the point part each
  iteration for stable coupling — **per-world, allocation-free, no statics** (VecEnv-safe).
  Baumgarte `kJointBaumgarte=0.2`, no slop. Stiff mass/inertia ratios (e.g. a point-mass on a
  long lever) need more iterations/substeps, as with contacts.
  **Limits (B2)**: Revolute min/max angle enforced as a one-sided constraint about the axis.
  **Actuators (B3)**: per Revolute joint, mode ∈ {`Torque`, `PDTarget(kp,kd,target,targetVel)`}
  + `maxTorque` clamp, applied about the hinge axis as an external angular impulse in the
  velocity-integration phase (RL torque semantics). **State read-path**: `JointState{q,qd}`
  (hinge angle/rate about the axis) via `jointState(h)` + bulk `jointStates()` span; **bulk
  action write** via `setJointTargets/Torques(span)` (batched observation/action for VecEnv).
  Tests: `tst/physics/unit/joints.cpp` (ball anchors coincide; hinge confines to axis + preserves
  on-axis spin; fixed holds a cantilever; hinge limit settles at the clamp; PD holds a target
  angle vs gravity; torque drives the joint; bulk write/read roundtrip).
  **Ball actuation (B4)**: 3-DOF spherical PD (`actuator.ballTarget`) or direct `ballTorque`,
  clamped to `maxTorque`, applied as a 3-DOF angular impulse. Multi-DOF ball `q/qd` for RL obs is
  derivable from the exposed body poses/`angularVelocities` (flat per-joint SoA is a follow-up).
- **Self-collision filtering (B4)**: `BodyDef::collisionCategory`/`collisionMask` (default = collide
  with all); a pair collides iff `(A.cat & B.mask) && (B.cat & A.mask)`. Articulation limbs share a
  category and mask it out so they don't fight their joints (still collide with the ground).
- **Optional velocity damping (B4)**: `WorldDef::linear/angularDamping` (default 0; drag applied to
  dynamic bodies each substep) so undamped joint DOFs (a ragdoll's free-spinning limb) settle.
- **Articulation (B4, `dynamics/articulation.h`)**: plain-data `ArticulationDef` (bodies +
  `JointSpec`s referencing bodies by index) → `buildArticulation(world)` → handles;
  `makeHumanoid()` preset = **~1.70 m** figure, **14 bodies** (pelvis, torso, shoulders, head,
  L/R upper+lower arms, L/R thigh+shin, L/R feet) + **13 joints** (ball waist/hips/shoulders,
  fixed chest/neck, hinge elbows/knees/ankles with limits). **Segments do not overlap** — each
  joint sits in the *gap* between two bodies (connected only by the constraint, an "invisible
  skeleton"). **Feet are the last two bodies** by contract so a renderer can draw them as boxes
  and the rest as stretched spheres (ellipsoids). Headless `tst/physics/integration/ragdoll.cpp`
  (passive ragdoll collapses + settles on the plane).
- **Humanoid behaviours (B5)**: `tst/physics/integration/humanoid_control.cpp` — `pd_stand_holds_pose`
  (pinned pelvis; PD servos drive bent knees against gravity + hold neutral elbows) and
  `articulation_determinism` (serial vs pooled/parallel bit-identical with joints+actuators).
  Visual: `tst/physics/visual/humanoid.cpp` — a gallery of three figures (free ragdoll +
  pelvis-pinned PD neutral hold + PD knee-bend hold; ankle gains scaled down to the light feet so
  they don't buzz). Limbs render as **capsules** matching their colliders, bulky bodies as
  stretched spheres, feet as boxes; fly camera. `core::geometry` gained `makeBox(halfExtents)` and
  `makeCapsule(radius, halfHeight)` (the box + capsule primitives the engine previously lacked).
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
**Articulation bridge (B4)**: `Joint{ JointHandle }` + `JointCommand{ target, torque }`
components; `actuatorFlushSystem` writes each joint's command to the world (run before
`stepSystem`); `spawnArticulation(ecsWorld, physicsWorld, ArticulationDef)` builds the bodies +
joints and mirrors them into the ECS (an entity per body with `RigidBody`+`Transform`, per joint
with `Joint`+`JointCommand`). Verified: `tst/physics/integration/articulation_ecs.cpp` (humanoid
built in ECS; actuator→step→sync schedule; Transforms track poses exactly, ragdoll settles).

### physics_env — RL env layer (`engine::physics_env`, Phase D, 2026-07-04)
Headless, **ECS-free** RL interface (deps `engine::physics` + `engine::core` only). The mechanism
for training; **no reward/termination/task** (those are downstream — the env exposes raw state).
- **`Environment`** (`physics_env/environment.h`): a thin layer over one `PhysicsWorld` + an
  `Articulation` (+ ground plane). `reset(seed)` (in-place: `setBodyState` to authored pose +
  `clearState` + zero actions + optional randomization hook + `refreshState`), `setAction(span)`
  (revolute 1 DOF, ball 3 DOF, fixed 0; `actDim=21` for the humanoid), `step()`
  (one control step = `substeps` physics substeps). **Action mode** (`EnvConfig::actionMode`):
  `Torque` (raw joint torques) or `PDTarget` (action = desired joint position — revolute angle /
  ball orientation as a rotation vector — tracked by a PD servo with `kp`/`kd`). **Raw-state
  accessors**: `jointStates()` (revolute q/qd + ball `rotation`/`angularVelocity`), `rootPose()`,
  root lin/ang velocity, per-body `bodyContactFlags()`. Downstream composes its obs from these; an
  **optional DOF-complete default packer** (`defaultObsDim()=69` for the humanoid: root pose 7 +
  twist 6 + joint positions[21] + velocities[21] + contacts[14]) is a convenience, not the contract.
- **`VecEnv`** (`physics_env/vec_env.h`): N independent `Environment`s (each a single-threaded
  `PhysicsWorld`) stepped across a `core::ThreadPool` via `parallelFor` (**no nesting** — the ML
  throughput lever is parallel *worlds*). Contiguous SoA batches: `actions()[N*actDim]` (caller
  writes), `observations()[N*obsDim]` (default packer). `reset(seed)` (per-env `seed+i`),
  `resetMasked(mask)`, `step()`.
- **In-place reset support** added to `PhysicsWorld`: `setBodyState`, `clearState`, `refreshState`,
  `setJointBallTorque`, `setJointBallTarget`.
- **D0 solver perf**: per-body world inverse inertia is now cached once per substep
  (`computeWorldInvInertia`) and read by the contact/joint/actuator/limit solves, instead of
  recomputing `R·I⁻¹·Rᵀ` per body per constraint per iteration — bit-identical (determinism tests
  still 0), win scales with contacts×iterations.
- Tests: `tst/physics_env/unit/environment.cpp` (dims 21/69, bounded random rollout, deterministic
  0.0), `tst/physics_env/integration/vec_env.cpp` (parallel==serial, bit-identical), benchmark
  `tst/physics_env/benchmark/vec_env.cpp` (N=1→1024; ~8.7k→68.7k env-steps/s on 13 workers).

### Differentiable engine (`engine::physics::diff`, Phase F, 2026-07-04)
Header-only, **Scalar-generic** (`double` for sim, `Dual<N>` forward-mode AD for gradients — same
templates), **quaternion-free** differentiable reduced-coordinate sim, for analytic-gradient RL
(SHAC / the α-order hybrid). Independent of the production backends; a separate "engine" alongside
the maximal/reduced `PhysicsWorld`s. Under `include/engine/physics/diff/`:
- **`dual.h`** forward-mode `Dual<N>`; **`linalg.h`** Scalar-generic V3/M3/V6/M6 spatial algebra +
  `expSO3`/`logSO3`/`vee`/`rodrigues`; **`articulated.h`** generalized ABA (`DiffModel`/`DiffState`,
  `diffForwardDynamics`/`diffSubstep`, `linkWorld`) for fixed/floating base + revolute/ball/fixed.
- **Contact**: smooth compliant ground — `k·softplus_β(pen)/β − c·vₙ·σ(β·pen)` + regularized Coulomb
  friction at the contact point (differentiable everywhere). **Multi-point** (`ContactSphere` list per
  link) with a per-collider **shape decomposition** (`DiffContact::{None,Feet,All}`: capsule→2 end-caps,
  box→8 corners) so shapes rest without clip-through. **`ContactIntegration::{Explicit,SemiImplicit}`**
  (SemiImplicit = IMEX: only the stiff contact force is treated implicitly, smooth dynamics stay
  explicit/symplectic). Normal force is **non-adhesive** (damping gated off on separation) and
  **physical joint damping** `jointDamping` is available (τ=−b·q̇, default 0). Defaults `groundK=4e4,
  C=1000` (stiff + over-damped ⇒ ~3 cm transient impact penetration, low bounce, no clip-through),
  `β=120, μ=0.8`. See notes/investigations/2026-07-04-diff-semiimplicit-testing.md.
- **`zeroth_order.h`** antithetic Gaussian-smoothing ES gradient; **`hybrid.h`** `alphaOrderGradient`
  (min-variance blend of analytic first-order + zeroth-order — Suh et al.); **`jacobian.h`** per-step
  tangent Jacobian `∂s_{t+1}/∂(s_t,a_t)` (exact `Dual<1>` per column, SO(3) exp/vee).
- **`from_articulation.h`** `articulationToDiffModel(ArticulationDef, DiffContact)` — converts the
  production `makeHumanoid` (or any articulation) to a `DiffModel` (inertia/anchors/axes/restRel/floating
  root); validated **==** the reduced backend to 9e-8 m. **`diff_environment.h`** `DiffEnvironment`
  mirrors `physics_env::Environment` (`reset`/`setAction`/`step`) + exposes `jacobian()` and
  `rolloutGradient<NA>()`. All gradients validated vs finite differences (≤1e-5, mostly 8+ digits).
  Forward substep ≈0.7× the reduced backend (faster; header-only + inlined). Design + hardening:
  investigations/2026-07-04-differentiable-reduced.md, -contact-geometry.md.

### Physics configuration (`engine/physics/config.h`, 2026-07-04)
Centralized, value-type (no global singleton) tuning config. **`SolverConfig`** un-buries the 10
formerly-hardcoded backend constants (Baumgarte/slop/maxCorrection/aabbMargin/jointBaumgarte;
reduced PGS iters/manifold cap/…). **`SimConfig`** = the single knob surface for a maximal/reduced RL
sim (gravity, controlDt, substeps, velocityIterations, damping, backend, ground, actuation
`maxTorque/actionMode/kp/kd`, `SolverConfig`); `WorldDef` derives via `toWorldDef()`, and
`EnvConfig = { ArticulationDef articulation; SimConfig sim; }` (one source of truth, no duplication).
**`SimConfigOverride` + `resolve(base, override)`** = sparse layered overrides; **`configs.h`** named
presets. **`config_io.h`** = write-only `serialize`/`configHash` (FNV-1a)/`dump` + `configVersion` for
per-run history (reader deferred until a training launcher). The **differentiable engine keeps its own
`DiffModel` config** (two-surface split; cross-engine unification deferred). Plan/decisions:
investigations/2026-07-04-physics-config-system.md.

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

## Input + controls (`engine::input`, `engine::input_glfw`, `engine::controls`, 2026-07-03)

Keyboard/mouse (Milestone 2, Phase A), split by dependency so the neutral layer is windowing-
backend-agnostic (GLFW today, Qt/other later) and headless/ML-friendly:
- **`engine::input`** (INTERFACE, header-only; deps core + glm, **GLFW-free**):
  `input::InputState` (`input/input.h`) — `Key`/`MouseButton` enums, level (`keyDown`) + edge
  (`keyPressed`/`keyReleased`) queries, mouse pos + delta, scroll. Mutators (`newFrame`,
  `setKey`, `setMouseButton`, `setMousePosition` (absolute), `addMouseDelta` (relative/event),
  `addScroll`) form an adapter contract that fits **both** polling (GLFW) and event-driven (Qt)
  backends, and scripted/ML input (fill it directly).
- **`engine::input_glfw`** (STATIC; deps input + glfw): `input::GlfwInput` — polls a GLFW window
  (passed as `void*`) into an `InputState`; scroll via a per-window callback registry; cursor
  capture. A future `engine::input_qt` sits alongside it.
- **`engine::controls`** (STATIC; deps ecs + input + core, **graphics-free**): input-driven ECS
  controllers. `controls::FlyController` component + `controls::flyControllerSystem` (a
  `void(World&)` system reading the `input::InputState` + `engine::Time` resources) drive an
  entity's `engine::Transform` (WASD/QE/sprint/RMB-look). The controller only touches
  `Transform` — it doesn't know the entity is a camera.
- **Camera as an ECS entity**: `engine::Camera` (`core/math/camera.h`) is render-agnostic
  *projection* data (perspective/ortho, fovY, near/far, `projectionMatrix(aspect)`; 0..1 depth
  via engine-wide `GLM_FORCE_DEPTH_ZERO_TO_ONE`). A camera entity = `Transform` (pose) +
  `Camera` (projection) + optional `FlyController`. `scene::extractViews` turns each
  `<Transform, Camera>` into a `render::RenderView` (view = inverse(pose)); the app fills each
  view's materials + target (invalid target ⇒ primary swapchain).
- `engine::Time` (`core/time.h`): frame-dt resource for variable-rate systems (physics keeps
  its own `FixedStep`).
- Drivers: `tst/input/unit/input_state.cpp` (edges/deltas/scroll), `tst/controls/unit/
  fly_controller.cpp` (ECS controller, headless), `tst/core/unit/camera.cpp` (projection);
  `tst/graphics/visual/input_demo.cpp` (windowed: camera-entity + Schedule[input-pump → fly] +
  extractViews + background/light toggles).

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
- **Environment** (`scene/environment.h`, Phase A): `Background` + `SceneLighting` ECS resources
  + `applyEnvironment(world, view)` copies them onto a `RenderView` (clearColor + directional
  light). The Renderer packs `RenderView::light` into a `GlobalUniforms` (viewProj + light) at
  binding 0; `mesh.slang` does ambient + Lambert shading (configurable, was hardcoded).
- **`scene::extractViews`** (Phase A): builds one `RenderView` per `<Transform, engine::Camera>`
  entity (view = inverse(pose), proj from Camera, environment applied); items/instances shared
  from `extract`. Camera-as-entity → maps onto RenderView's multi-view design.

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
