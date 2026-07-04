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

### Phase B — Articulated physics (the core) — APPROVED PLAN (2026-07-03), grounded in the solver

Maximal-coordinate **joint constraints + actuators** on the existing sequential-impulse solver
(decision: [2026-07-03-articulation-approach.md](2026-07-03-articulation-approach.md)). Reviewed
against `notes/core/goals.md` (games / ML-parallel / 100k / determinism / plain-data boundary) —
holds up; see "Alignment + 3 refinements" below.

**How joints fit the existing solver** (`src/physics/backends/realtime/sequential_impulse_world.cpp`):
per-substep loop is (1) integrate velocities (gravity) → (2) `buildConstraints` (contacts, rebuilt
each step) → (3) N velocity iterations of `solveColored()` → (4) integrate positions/orientations
(exp-map). Joints are velocity constraints in the **same loop**, but **persistent** (created once,
alive across steps) — so they get a persistent store + **cross-step warm-starting** (contacts
don't). Reuses `effectiveMass`/`applyImpulse`/`worldInvInertia` and the Baumgarte push-out style.

- **B0 Design doc** ✅ DONE (articulation-approach.md; constraints-first, reduced-coord deferred).
- **B1 Joint constraints core** (this pass): `JointHandle`/`JointType{Ball,Revolute,Fixed}`/
  `JointDef` (bodies + local anchor per body + local hinge axis per body) in `world.h`;
  `createJoint`/`destroyJoint` virtuals. Backend: persistent `joints_` vector (index-stable,
  free-list) with warm-start accumulators; **serial** solve in the iteration loop (before the
  colored contacts), deterministic in creation order. Constraints: **ball** = point-to-point (3,
  3×3 K-matrix + Baumgarte from world-anchor error); **hinge** = point-to-point + 2 angular
  (kill relative ω off the hinge axis, Baumgarte from axis misalignment); **fixed** =
  point-to-point + 3 angular (lock relative orientation to the creation-time reference).
  **Per-world, allocation-free per step, no mutable statics** (refinement 2). Unit tests: ball
  anchors coincide (no drift under gravity), hinge off-axis relative rotation ≈ 0, fixed keeps
  relative pose.
- **B2 Joint limits**: hinge min/max angle as a **one-sided** constraint (like a contact at the
  limit). Test: swings to and stops at the clamp.
- **B3 Actuators + state read**: per-DOF mode ∈ {`Torque`, `PDTarget(kp,kd,targetPos,targetVel)`}
  + torque limit, applied as **external angular impulses in the velocity-integration phase**
  (RL torque semantics); explicit PD + substepping first (SPD if jittery). Read-path: joint
  `q`/`qd` (single + **bulk SoA span**, mirroring `poses()`), **and bulk SoA actuator write**
  (refinement 1 — batched action tensor, not just per-joint setters). Backend-agnostic so the
  future Featherstone backend serves the same `q/qd`/torque API. Tests: PD "stand" holds a target
  angle vs gravity; torque mode → expected angular accel.
- **B4 ECS bridge + articulation builder + humanoid preset + self-collision filtering**:
  `Joint{JointHandle}` component (joint-as-entity) + `JointActuatorCommand` flush system (where
  actions land); data description (bodies/joints/actuators) → spawns entities + world bodies +
  joints; humanoid preset (~15 capsule/box limbs; ball hips/shoulders, hinge knees/elbows/
  ankles). **Self-collision filtering** (REQUIRED): a `collisionGroup`/`mask` on `BodyDef`
  consulted in the candidate-pair filter so jointed parent↔child limbs don't fight the joint.
- **B5 Tests + demo**: passive **ragdoll** collapses & rests on the flat plane (no explosion,
  |ω|→0, energy non-increasing); scripted **PD "stand"** holds a pose; **determinism** (bit-
  identical serial vs parallel — joints serial in creation order + contacts colored). Visual:
  ragdoll / PD-stand on the plane with the fly camera.

**Alignment + 3 refinements (reviewed vs core goals):**
1. **Bulk SoA actuator write + q/qd readback** (B3) — matches "batched observation/action
   tensors"; per-joint setters kept for interactive/game use.
2. **Per-world, allocation-free, no-statics joint solve** (B1) — ML throughput is *across worlds*
   (VecEnv, Phase D), not intra-world; a 15-body humanoid steps single-threaded and that's
   correct. Warm-start impulses live in the persistent `joints_` vector (no per-step heap churn);
   use per-world member scratch (not `thread_local`) so concurrent worlds are unambiguously safe.
3. **Maximal-coordinate compromise (known, mitigated)**: more solver DOF/drift per humanoid than
   reduced coords, but the `q/qd` accessor still exports the *minimal* observation (read joint
   angles/rates, not redundant body poses), and the reduced-coordinate Featherstone backend
   (deferred) is the throughput/stability/gradient answer — a transparent backend swap via the
   backend-agnostic API. 100k free bodies unchanged (joints are additive); many ragdolls in one
   giant world → fold joints into graph-coloring (escape hatch).

### Phase C — Terrain ⏸ DEFERRED (2026-07-03), revisit after B/D
A flat `Plane` is enough to get the humanoid balancing + walking; varied terrain is a
refinement, not a prerequisite. Full design + rationale + the "GJK/EPA ≠ arbitrary mesh
collider" analysis: [2026-07-03-terrain-collision-deferred.md](2026-07-03-terrain-collision-deferred.md).
Revisit when locomotion needs slopes/stairs/rough ground (e.g. an RL terrain curriculum).
- (deferred) C1 Heightfield collider (sphere→capsule; AABB-reject broadphase like Plane).
- (deferred) C2 Procedural generation in `core::geometry` → heightfield + render mesh.
- (deferred) C3 Render terrain as a lit static mesh; body-settles-on-terrain test.

### Phase D — RL-ready env interface (depends on B+C, ECS command buffer)
Mechanism only — reward/task/training live downstream. **Detailed reviewed plan:**
[2026-07-04-phase-d-plan.md](2026-07-04-phase-d-plan.md). The headless `Environment` drives a
`PhysicsWorld` **directly (ECS-free)**, so the ECS command buffer originally listed here is **not
needed** for RL (reset happens between steps). Sub-phases:
- **D0 Solver perf micro-opt**: cache per-body world inverse inertia once per substep (contacts +
  joints read it) instead of recomputing per-constraint-per-iteration. Bit-identical; measure.
- **D1 `Environment`** (ECS-free): `reset(seed)`/`setAction(span)`/`step()`/`observe(span)`; fixed
  obs (root pose+twist + joint q/qd + foot contacts) / action layout; no reward/termination (hooks
  only). In-place state reset preferred over rebuild.
- **D2 `VecEnv`**: N single-threaded worlds on the `ThreadPool` (`parallelFor`, no nesting);
  batched SoA `obs[N×obsDim]`/`act[N×actDim]` (contiguous, zero-copy for downstream C/Python).
- **D3 Determinism + throughput review**: same seed+actions ⇒ identical rollout; random-rollout
  test; env-steps/sec benchmark; re-evaluate "engine is enough" → spin up downstream repo.

### Phase E — Reduced-coordinate Featherstone backend (own track, after D)
Split out of D so it can slip without blocking RL-readiness. A second `PhysicsWorld`
(`Backend::Reduced`) behind the same Environment/obs-action API. Built incrementally: **E0** ABA
spatial-algebra core (no contacts) → **E1** contact coupling (the hard part) → **E2** humanoid +
actuators → **E3** behind VecEnv. Rationale + decision:
[articulation-approach.md](2026-07-03-articulation-approach.md); details in the Phase D plan doc.
Differentiability is a further extension, later.

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
1. ~~**Phase A** (input + lighting + background)~~ ✅ DONE (2026-07-03).
2. ~~**B0 design doc** (articulation decision)~~ ✅ DONE — constraints-first, reduced-coord deferred
   ([2026-07-03-articulation-approach.md](2026-07-03-articulation-approach.md)).
3. **Phase B** (joints → actuators → humanoid) — the long pole; **next up**. On flat ground.
4. ~~Phase C (terrain)~~ ⏸ **DEFERRED** — flat `Plane` suffices for now; revisit after B/D.
5. **D0 ECS command buffer** — can start anytime; needed before the rest of D.
6. **Phase D** (env → vec-env → obs/action → determinism) — integrates B (on flat ground).
7. Re-evaluate: is the engine "enough"? If yes, **spin up the downstream sim repo**. (Fold terrain
   back in when locomotion needs varied ground.)
