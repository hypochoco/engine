# Phase D plan — RL-ready env interface (+ perf micro-opt); Phase E — Featherstone backend

Plan for review (2026-07-04). Milestone 2's final env-side phase (**Phase D**): turn "we can
simulate an actuated humanoid" into "a downstream repo can train on it." Also covers the
joint-solve perf micro-opt. The reduced-coordinate **Featherstone** backend is split into its own
**Phase E** (its own slip-able track — it must not block the RL-ready deliverable).

Living status: `notes/core/todo.md` (Phase D/E backlog) + milestone plan
([2026-07-03-humanoid-rl-milestone-plan.md](2026-07-03-humanoid-rl-milestone-plan.md) §Phase D/E).

## Guiding constraints (from goals.md + what we've built)

- **Engine = mechanism only.** No reward/termination/task/curriculum/RL-algorithm — those live in
  the **downstream repo**. The env exposes raw state + takes actions; the boundary must stay
  plain-data and task-free.
- **Throughput comes from parallel *worlds*, not intra-world threads.** `ThreadPool::parallelFor`
  is blocking and **must not nest**, so a `VecEnv` runs N **single-threaded** `PhysicsWorld`s, one
  per pool task. (A humanoid is ~14 bodies — intra-world threading does nothing for it anyway.)
- **Determinism** per env (same seed + actions ⇒ identical rollout). Already true for the solver +
  joints + actuators (bit-identical serial vs parallel); Phase D must preserve it end-to-end.
- **Backend-agnostic API.** The joint/actuator + obs/action surface (bulk SoA `jointStates` /
  `setJointTargets`/`setJointTorques`, `poses`) was built to be identical across backends, so the
  Featherstone backend drops in behind the same `Environment`.
- **Headless is first-class.** The `Environment` drives a `PhysicsWorld` **directly, ECS-free**
  (leaner, no archetype overhead); the ECS bridge stays for interactive/rendered scenes. ⇒ the
  originally-listed "D0 ECS command buffer" is **not needed** for headless RL (reset happens
  between steps, not inside a system iteration).

## Sub-phases

### D0 — Solver perf: cache per-body world inverse inertia (the micro-opt)
Today `solveConstraint` (contacts) and `solveJoints` recompute `worldInvInertia(orientation,
invInertiaLocal)` (a quat→mat3 + two mat3 muls) for **each body, each constraint, each velocity
iteration**. Orientation is constant during a substep's velocity solve, so this is redundant.
- Compute a per-body `worldInvInertia` **once per substep** into a scratch array (indexed by body),
  after gravity/actuator integration and before the iteration loop; have the contact solver, joint
  solver, actuator, and limit paths read from it.
- Also fold the earlier joint note: cache `IinvA/IinvB` for joints from the same array.
- **Verify**: bit-identical results (determinism test must still pass) + measure the speedup on
  `tst/physics/benchmark/step.cpp` (stacks/contacts) and the humanoid ragdoll. Self-contained; do
  first as a warm-up. Expected: meaningful per-step reduction at scale (fewer mat3 ops per iter).

### D1 — `Environment` (headless, ECS-free, over PhysicsWorld + Articulation)
Lives in a **new `engine::physics_env` module** (mirrors `engine::physics_ecs`; deps
**`engine::physics` + `engine::core`**, never `engine::ecs` — so "ECS-free" is enforced at the
dependency level). Owns a `PhysicsWorld` + an `Articulation` (built from an `ArticulationDef`, e.g.
`makeHumanoid`).
- **API** (plain data, no task assumptions):
  - `reset(uint64_t seed)` → deterministic start state (in-place, see below); optional
    randomization *hook* (`std::function`) supplied by the caller — engine ships none.
  - `void setAction(std::span<const float>)` → maps the flat action vector onto actuator commands
    (`setJointTargets`/`setJointTorques`, per the actuation mode chosen at construction).
  - `void step()` → advance one **control step** = `controlDecimation` physics steps at the fixed dt.
  - Raw-state accessors → `jointStates()` (q/qd), root pose+twist, `contactFlags()`. See below.
  - `size_t actDim()` (fixed for a given articulation + config).
- **Observation policy — composition deferred downstream (resolved).** Composing an obs *vector*
  (which fields, normalization, history-stacking) is a task/policy decision ⇒ downstream. So the
  engine **exposes the raw state** (joint `q/qd` via `jointStates`, root pose/twist, foot contact
  flags — all already available from `PhysicsWorld`) and downstream assembles its own obs. The
  engine ships an **optional default flat packer** (root pos 3 + quat 4 + lin vel 3 + ang vel 3 +
  per-DOF q + per-DOF qd + contact flags) as a convenience for tests/examples only — not the
  mandated contract. **No reward/termination** anywhere in the engine.
- **Action mapping**: the engine defines the fixed **action-index → joint-DOF** mapping (it knows
  the joints: hips/shoulders ball ×3, knees/elbows/ankles hinge ×1, waist) and applies raw values;
  the *values/normalization* are downstream. `actDim` is fixed per articulation + actuation mode.
- **Reset strategy (resolved): in-place.** Restore each body's pose/vel to the initial value, zero
  the joints' warm-start accumulators + actuator commands, clear contact state — no destroy/recreate.
  Needs a small `PhysicsWorld` addition (e.g. `resetBody(handle, pose, vel)` + a joint/contact
  clear, or a bulk `resetState`). The env stores the initial state from the `ArticulationDef`.
- **Tests**: build a humanoid env; `actDim` + raw-state sizes correct; state finite after a random
  action rollout; **reset(seed) is deterministic** (two envs, same seed + action sequence ⇒
  identical state trajectories); the default packer round-trips.

### D2 — `VecEnv` (batched, parallel)
N independent `Environment`s stepped in parallel (in `engine::physics_env`).
- Owns N envs (each a single-threaded `PhysicsWorld`) + **batched raw-state SoA buffers**: batched
  `q/qd`, batched root pose/twist, batched contact flags, plus `act[N*actDim]`, `done[N]`.
  Contiguous `float*`/span views → **downstream composes obs from these** (or uses the default
  packer) and consumes them zero-copy from a C-ABI / Python (Gym-style vectorized env) layer.
- `step()`: `pool.parallelFor(N, ...)` — each task calls `env[i].setAction(act row i)`,
  `env[i].step()`, then writes env i's raw state into the batched buffers. No nesting (each env
  single-threaded) → safe + deterministic (disjoint envs, no shared writes).
- `reset(mask)`: reset a subset (episode boundaries), also parallel.
- **Tests**: N-env batched step == running each env serially (per-env determinism); throughput
  benchmark (env-steps/sec vs N, showing parallel-worlds scaling); memory is O(N) SoA.

### D3 — Determinism + throughput review
- End-to-end: same seed + same action stream ⇒ identical rollout, serial vs pooled, across the
  batch. (Extends the existing joint/actuator determinism to the env layer.)
- A "random-policy rollout" integration test (headless): step a `VecEnv` of humanoids for K control
  steps with scripted pseudo-random actions; assert finite, bounded, reproducible.
- Benchmark: env-steps/sec at N ∈ {1, 64, 1024}; record in the benchmark suite (Release).
- **Re-evaluate**: is the engine "enough" for the downstream sim repo? (Per goals.md, once D lands
  the task/reward/RL/cloud layer moves to a separate repo consuming this engine.)

### D4 — Reduced-coordinate Featherstone backend
Moved to its own **Phase E** (below) so it can proceed independently and slip without blocking the
RL-ready deliverable (D0–D3).

## Phase E — Reduced-coordinate Featherstone backend (its own track)

A **second `PhysicsWorld` backend** (`Backend::Reduced`; `backends/reduced/featherstone_world.cpp`)
implementing the same interface, validated behind the D1–D3 `Environment`/`VecEnv`/obs-action API
to prove backend-agnosticism. Rationale (throughput + stability + cleaner gradients) is in the
[articulation approach note](2026-07-03-articulation-approach.md).

**Why a separate phase, after D:** the env layer + RL-readiness (the milestone deliverable) must
not depend on Featherstone landing. Build/prove the env on the working maximal-coordinate backend
(Phase D), then add Featherstone as a drop-in and run the *same* VecEnv on both. Phase E is the
single largest piece of the whole milestone (spatial algebra, per-joint motion subspaces, and
especially contact coupling) — a slip-able track that doesn't gate D.

**Incremental build (each step independently testable):**
- **E0 — Spatial algebra + ABA core (no contacts).** Reduced state = floating-base root (6 DOF)
  + generalized `q`/`q̇` per joint DOF; per-joint motion subspaces (revolute 1, ball 3, fixed 0,
  free 6). Articulated-Body Algorithm in O(n): 3 sweeps (fwd velocity/bias-force, backward
  articulated-inertia, fwd acceleration); `τ` is a direct generalized-force input. Integrate `q̇`
  (semi-implicit) + root via SE(3) exp map. **Validate** a free-floating chain/pendulum vs the
  maximal backend (same gross motion; energy behaviour sane — not bit-identical, different method).
- **E1 — Contact coupling (the hard part).** Reuse the existing collision/broadphase substrate
  (it already emits `Contact`s). Couple contacts back to the reduced system: compute contact-space
  inertia from ABA quantities and solve an LCP/PGS in contact space (Bullet/PhysX-style), or a
  soft-constraint convex formulation (MuJoCo-style). **Validate** a sphere/box resting + a body
  settling on the plane.
- **E2 — Humanoid + actuators.** Map `makeHumanoid` (or a reduced description) onto the reduced
  model; joint `q/qd` **are** the state (no derivation); PD/torque actuation as generalized force.
  **Validate**: ragdoll settles, PD-stand holds — same qualitative behaviour as maximal.
- **E3 — Behind VecEnv.** Run the D2 `VecEnv` with `Backend::Reduced`; confirm the obs/action API
  is unchanged and per-env determinism holds; benchmark vs maximal (expect smaller obs + steadier
  O(n) step).

**Out of scope even for E:** differentiability (analytic gradients) is a *further* extension on top
of the reduced model — a later phase if pursued.

## Out of scope for Phase D (deferred, consistent with prior decisions)
- Reward / termination / task / curriculum / RL algorithms / Python bindings / cloud — **downstream repo**.
- Terrain (Phase C, deferred) — envs run on the flat plane.
- Differentiable physics — a later accelerator on top of the reduced backend.

## Decisions — RESOLVED (2026-07-04)
1. **`Environment` is ECS-free** (drives `PhysicsWorld` directly); ECS bridge stays for
   interactive/render use. Rendering an env is an opt-in adapter. ✓
2. **Reset = in-place state reset** (small new `PhysicsWorld` reset path; no destroy/recreate). ✓
3. **Observation composition deferred downstream** — engine exposes raw batched state (q/qd, root
   pose+twist, contact flags) + an *optional* default flat packer for tests/examples; downstream
   composes the obs vector. Action-index→joint-DOF mapping is engine-side, values are downstream. ✓
4. **Reward/termination stay downstream**; env exposes only raw state + optional caller-supplied
   randomization/termination *hooks*. ✓
5. **New module `engine::physics_env`** (mirrors `engine::physics_ecs`; deps physics + core, not
   ecs — ties into #1). ✓
6. **Phase E (Featherstone)** is its own track after D0–D3, built incrementally E0→E3. ✓
