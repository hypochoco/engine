# Milestone 2 plan — "a physics humanoid walking on terrain" (RL-ready)

Point-in-time planning doc (2026-07-03). Maps the next milestone against current features and
lays out a phased plan. Living status lives in `notes/core/{goals,todo}.md`; this doc is the
reasoning behind it and won't be kept in sync.

## The milestone

> An **articulated, physically-simulated humanoid** — jointed limbs, actuated, affected by
> gravity/contact/friction — that can be **driven interactively (keyboard/mouse) or
> programmatically (an action vector)**, standing and walking about **procedural terrain**,
> rendered with **basic lighting + a controllable background**, and **steppable headless in
> parallel batches** exposing **batched observation/action tensors**.

This is the *engine-side* target: the mechanism and infrastructure to make an RL locomotion
task possible. It deliberately **stops short of training**. The RL algorithm, reward shaping,
curriculum, task definitions, Python bindings, and cloud job orchestration live in a **separate
downstream repo** that pulls this engine in as a dependency (see "The split" below). The MuJoCo
"Humanoid-v4" / DeepMind Control "walker/humanoid" benchmarks are the mental model for scope.

Why this milestone: it forces the systems that Milestone 1 ("ball rolling down a plane") left
open — articulated dynamics + actuation, non-flat collision (terrain), input, lighting, and a
vectorized headless env interface — which together are the foundation for everything downstream
(locomotion RL, imitation, cloud training).

## What we already have (the foundation)

Milestone 1 left us in a strong position:

- **ECS** (archetype `World`, queries, resources, ordered `Schedule`). Deterministic iteration.
- **Physics** (`engine::physics`, Phase 0–2 + polish): rigid-body dynamics with **rotation**
  (SO(3) exp/log), shapes (sphere/plane/box/hull/**capsule**), full collision (GJK/EPA +
  distance, face-clip manifolds, stacking), **sequential-impulse solver with Coulomb friction**
  (true rolling), clamped Baumgarte, **CCD**, **kinematic bodies**, restitution.
- **Broadphase** that scales (uniform grid, ~30 ms/step @ 100k bodies single-threaded).
- **Parallel physics worlds** (7.7× on 12 workers) + **deterministic** intra-world threading
  (bit-identical serial vs parallel). ← the seed of vectorized RL envs.
- **Runtime-virtual multi-backend `PhysicsWorld`** (`createPhysicsWorld(Backend,...)`) — designed
  so a new articulated/differentiable backend can be added alongside the impulse backend.
- **Graphics** (RHI + Metal): instanced rendering, per-instance materials (color), depth,
  offscreen + windowed. `core::Vertex` already carries **normals**; `InstanceData` carries a
  **normal matrix**; the fragment shader already does a (hardcoded) directional-light term;
  `RenderView` already has a (configurable but unsurfaced) `clearColor`.
- **Scene bridge** (`scene::extract`: ECS → `RenderView`).
- **Threading** (`ThreadPool`, `parallelFor`, `parallelSort`) and a **test harness**.

## Gap analysis (have → need)

| Area | Have | Need for milestone |
|---|---|---|
| **Input** | GLFW window (windowed tests only) | `engine::input` module: keyboard/mouse state, edge detection, mouse delta/scroll; headless-safe no-op; ECS resource. Camera controller. |
| **Lighting** | hardcoded dir light in `mesh.slang`; `clearColor` field exists | A real **light/scene uniform** (dir, color, intensity, ambient) surfaced through `RenderView`/scene; **background color** controllable at the scene/ECS level. |
| **Articulation** | free 6-DOF rigid bodies only | **Joints/constraints** (ball, hinge, limits) + **actuators** (PD/torque motors) + an **articulated-body builder** + a **humanoid preset**. *(Design decision below.)* |
| **Terrain** | infinite `Plane` only | **Heightfield collider** + narrowphase (sphere/capsule/box vs heightfield) + **procedural terrain generation** (feeds collider + render mesh) + render it. |
| **Env interface** | parallel worlds, determinism | **`Environment`** abstraction (reset/step, headless) over batched worlds; **SoA observation extraction** + **action application**; determinism review. |
| **ECS enablers** | spawn/destroy, query, schedule | **Command buffer** + **add/remove-component** (build/reset episodes cleanly); optionally ECS-level parallel worlds. |
| **Rendering rollouts** | offscreen tests exist | Clean **headless device / `Swapchain`↔`Renderer` split** (already a backlog item) for recording rollout video offline. Nice-to-have, not blocking. |

Notably **not** needed for this milestone (defer): bindless textures, Vulkan-behind-RHI port,
compute pipelines, differentiable physics (only needed for analytic-gradient RL, not PPO/SAC).

## Key design decision: articulation approach

The one consequential fork. A humanoid is a kinematic tree of ~15 bodies and ~20 actuated DOFs.

**Option A — Maximal coordinates + constraints** (Box2D/Bullet-style). Each limb stays a free
6-DOF body; joints are bilateral **constraints** solved by the existing sequential-impulse
solver (add constraint rows: point-to-point, hinge axis, angular limits; warm-start them).
Actuation = motor constraints / applied torques.
- Pros: **minimal new machinery** — reuses the solver, contacts, friction, threading,
  determinism we already have; fastest path to a visible, controllable ragdoll; incremental.
- Cons: constraint **drift/softness** under stiff chains + high mass ratios; needs many
  iterations; contact + tall articulations can be finicky to tune (mitigations: warm-starting,
  soft/TGS constraints, more substeps).

**Option B — Reduced coordinates (Featherstone / Articulated-Body Algorithm)** (MuJoCo / DART /
PhysX-articulation / Isaac Gym-style). The joint coordinates *are* the DOFs; O(n) forward
dynamics; no constraint drift by construction; joint torques are the natural action space;
cleaner path to **differentiability** (ties to deferred Phase 3).
- Pros: **rock-solid stiff chains**, the substrate real humanoid-RL uses; better numerical
  behavior for control; differentiable-friendly.
- Cons: **much larger build** (spatial algebra, ABA, contact coupling as constraints on the
  reduced system via LCP/soft-constraints); a distinct sub-project.

**Recommendation (proposed, decide before Phase B):** sequence them.
1. Build **Option A first** — joints + motors on the existing solver — to get an actuated,
   rendered, controllable humanoid end-to-end quickly. This unblocks input, terrain, and the
   env interface (Phases A/C/D) against a real articulated body.
2. Then add **Option B as a second `PhysicsWorld` backend** (the multi-backend factory already
   exists for exactly this) for RL-grade stability + eventual differentiability. Design the
   Phase-D env interface to be **articulation-backend-agnostic** (it reads joint `q/qd` and
   writes actuator targets/torques, not caring which backend produced them), so switching later
   is transparent to downstream training code.

This deserves its own focused design doc (à la `2026-07-03-physics-plan.md`) before committing.

## Phased plan

Ordering favors "make it visible early, then deepen." A/C are largely independent of B and can
run in parallel; B is the long pole; D depends on B+C and the ECS command buffer.

### Phase A — Interaction + graphics basics (small; do first)
Unblocks watching/driving everything and is explicitly requested.
- **A1 `engine::input`**: poll GLFW into an `InputState` (keys down/pressed/released this frame,
  mouse pos/delta, buttons, scroll); expose as an ECS **resource**; headless build is a no-op
  stub. Driver: a visual test where keys move a cube/camera.
- **A2 Camera controller**: fly + orbit camera driven by A1 (for observing the humanoid).
- **A3 Lighting**: promote the hardcoded shader light to a **`SceneLighting`** uniform
  (directional dir/color/intensity + ambient); thread it through `RenderView` → shader.
- **A4 Background**: surface `RenderView::clearColor` at the scene/ECS level (a `Background`
  resource/component). Verify visually + a pixel test.

### Phase B — Articulated physics (the core)
- **B0 Design doc**: settle Option A vs B (see above); define joint types + actuator model.
- **B1 Constraints**: add bilateral **joint constraints** to the impulse solver — **ball/socket**
  (point-to-point), **hinge/revolute** (axis + angular limits), **fixed/weld**; warm-start;
  clamp like contacts. Unit tests (double pendulum energy behavior, hinge limit, chain rest).
- **B2 Actuators**: per-joint **PD servo** (target angle/velocity, torque limit) + direct torque
  control. This is the RL **action** surface.
- **B3 Articulation model + builder**: a data description (bodies = capsules/boxes, joints
  {type, frames, axis, limits}, actuators) + a programmatic builder that instantiates it into a
  `PhysicsWorld`. Consider URDF/MJCF import **later** (downstream).
- **B4 Humanoid preset**: torso/head/upper+lower arms/upper+lower legs/feet as capsules+boxes,
  ball hips/shoulders, hinge knees/elbows/ankles.
- **B5 Tests**: passive **ragdoll** collapses & rests on the ground (no explosion, |ω|→0); a
  scripted **PD "stand"** holds a pose; determinism preserved (bit-identical serial vs parallel).

### Phase C — Terrain (parallel with B)
- **C1 Heightfield collider**: `Heightfield` shape (grid of heights + spacing) + narrowphase
  (sphere/capsule/box vs the local triangles/cells); reuse manifold generation.
- **C2 Procedural generation**: terrain generators in `core::geometry` (slopes, stairs, gaps,
  value/Perlin noise) producing **both** the heightfield (collision) and a render `MeshData`.
- **C3 Render**: draw the terrain as a lit static mesh; a visual test of a ball/capsule rolling
  and settling on rough terrain.

### Phase D — RL-ready env interface (depends on B+C, ECS command buffer)
Mechanism only — reward/task/training live downstream.
- **D0 ECS command buffer + add/remove-component** (also a standing ECS backlog item): needed to
  build and **reset** episodes (spawn/despawn humanoid + terrain entities) deterministically.
- **D1 `Environment` abstraction**: headless `reset()` / `step(actions)` over a `PhysicsWorld`
  (+ optional offscreen render for video). No reward/termination baked in — those are
  **callbacks/hooks** the downstream repo supplies.
- **D2 Vectorized envs**: `VecEnv` of N independent `Environment`s stepped on the `ThreadPool`
  (builds directly on the existing parallel-worlds path). Batched, SoA.
- **D3 Observation/action tensors**: extract per-env **SoA float buffers** — joint `q/qd`, root
  pose/velocity, contact flags, terrain samples → `obs[N × obsDim]`; apply `act[N × actDim]` →
  actuator targets/torques. This is the binding-friendly contract downstream code consumes.
- **D4 Determinism review**: same seed + actions ⇒ identical rollouts across the batch (fixed
  step, stable iteration/coloring order — mostly already true; verify end-to-end).

## The split (engine vs downstream simulation repo)

Once Phases A–D land, the engine has "enough." At that point the **full simulation moves to a
new repo** that consumes this engine as a dependency. Rough division of responsibility:

**Stays in the engine (this repo):**
- Input, lighting, articulated physics + actuators, terrain, rendering.
- The generic `Environment` / `VecEnv` mechanism + batched obs/action buffers.
- Determinism, parallelism, headless operation.

**Moves downstream (new repo):**
- Concrete **tasks** (walk/run/navigate), **reward** functions, **termination**, curriculum.
- **RL algorithms** (PPO/SAC/…), networks, the training loop.
- **Python bindings** (a C ABI over `VecEnv` if we want a Gym-style vectorized env in Python).
- **Cloud** job orchestration, distributed rollout/learner, checkpoints, experiment tracking.
- URDF/MJCF assets + importers if we go the asset route; research-paper reproductions.

Design consequence for the engine: keep the env/articulation/obs-action API **clean,
stable, and free of task/policy assumptions** so the downstream repo can build on it without
reaching into internals. Prefer plain-data (SoA float spans, POD structs) at the boundary.

## Open questions (decide as we reach them)
- **Articulation approach** (A vs B) — the big one; needs its own design doc before Phase B.
- **Actuation model**: torque vs PD-target vs muscle — start torque+PD; RL usually uses one of
  these as the action space.
- **Terrain representation**: heightfield (recommended, matches locomotion RL) vs general
  triangle-mesh collider (more general, more work). Could add tri-mesh later.
- **Env↔engine boundary shape**: header-only C++ interface now; add a C ABI when the downstream
  Python bindings are built (probably downstream).
- **Many envs in-process vs multi-process** (already parked in todo "Compute/ML"): in-process
  vectorized (`ThreadPool`) first; multi-process/distributed is a downstream/cloud concern.
- **Differentiable physics**: still deferred; only needed for analytic-gradient methods, not
  model-free PPO/SAC. Reduced-coordinate (Option B) would make it cleaner if pursued.

## Suggested sequencing
1. **Phase A** (input + lighting + background) — quick, high leverage, makes everything watchable.
2. **B0 design doc** (articulation decision) in parallel with A.
3. **Phase B** (joints → actuators → humanoid) — long pole.
4. **Phase C** (terrain) — overlaps B.
5. **D0 ECS command buffer** — can start anytime; needed before the rest of D.
6. **Phase D** (env → vec-env → obs/action → determinism) — integrates B+C.
7. Re-evaluate: is the engine "enough"? If yes, **spin up the downstream sim repo**.
