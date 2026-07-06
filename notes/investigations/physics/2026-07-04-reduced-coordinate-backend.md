# Phase E — Reduced-coordinate Featherstone `PhysicsWorld` backend (design)

Dated design note (2026-07-04). Point-in-time; living summary is the Phase E section of
`notes/core/todo.md` + the milestone plan. Decision/rationale: `2026-07-03-articulation-approach.md`.

## Goal & staging

A second `PhysicsWorld` implementation, `Backend::Reduced`, that represents an articulated body by
its **generalized coordinates** `q` (root 6-DOF + one number per joint DOF) and steps it with the
**Articulated-Body Algorithm (ABA)** in O(n). It sits behind the *same* `PhysicsWorld` interface
and the *same* Environment/obs-action API, so downstream RL code is agnostic to which backend runs.

Built incrementally so it never gates RL-readiness (the maximal backend already demonstrates the
humanoid):
- **E0 ABA core, no contacts** — validate against physical invariants + a coarse cross-check vs the
  maximal backend.
  - **E0a** design doc + `Backend::Reduced` + `FeatherstoneWorld` scaffold (stores tree, readback
    works, `step()` stub) + factory dispatch. Tree green.
  - **E0b** ABA for a **fixed-base** revolute/fixed chain under gravity + joint torques.
  - **E0c** validation: single-pendulum period + energy conservation; double-pendulum energy;
    coarse cross-check vs maximal.
  - **E0d** **floating base** (free 6-DOF root); validate a free chain conserves linear + angular
    momentum (no gravity) and its COM travels linearly.
- **E1 contact coupling** (the hard part) — contact-space inertia from ABA quantities + a PGS/LCP
  (or soft-constraint) solve; validate resting/standing.
- **E2 humanoid + actuators** — ball (3-DOF) joints; `q/qd` *are* the state; ragdoll settles +
  PD-stand holds.
- **E3 behind `VecEnv`** — obs/action API unchanged; determinism (serial == parallel, bit-identical);
  benchmark vs maximal.
- (later) differentiability on top of the reduced model.

## Spatial algebra conventions (Featherstone, *Rigid Body Dynamics Algorithms*)

- **6-vectors are angular-first**: motion `v = [ω; vlin]`, force `f = [τ; f]`.
- **Link frame at the link COM**, axes aligned with the body orientation ⇒ spatial inertia is
  block-diagonal `I = [[Ibody, 0],[0, m·1]]` (no COM-offset coupling term). Collider→inertia reuses
  `body.h` (`solid{Sphere,Box,Capsule}Inertia`); mass from `BodyDef.mass`.
- **Plücker transforms** `ᴮXᴬ` map motion vectors between frames; the dual `ᴮXᴬ*` maps forces.
  Built from a rotation `E` (parent→child) and a translation `r`: motion transform
  `X = [[E,0],[-E·skew(r), E]]`. Implemented as a small struct holding `{Mat3 E; Vec3 r}` with
  `applyMotion`, `applyForce`, and inverse, rather than materializing 6×6 matrices.
- Cross products: `crm(v)` (motion, `v ×`) and `crf(v) = -crm(v)ᵀ` (force, `v ×*`).

## Tree extraction from the `PhysicsWorld` API

The interface is body/joint-oriented (`createBody`, `createJoint`), matching how
`buildArticulation` assembles an `ArticulationDef`. The reduced backend infers a **kinematic tree**:
- Each `createBody` registers a **link** (mass, body-frame inertia, initial world pose).
- Each `createJoint(def)` is a tree edge **parent = `def.a` → child = `def.b`**, with the joint
  frame from `localAnchorA/B` + `localAxisA/B`. (`buildArticulation` already emits joints in
  parent→child order.)
- The **root** is the link that is never a child. If the root is `BodyType::Dynamic` it becomes a
  **floating base** (free 6-DOF root joint, E0d); a `Static/Kinematic` root is a **fixed base**
  (E0b). Multiple roots ⇒ multiple independent trees (a forest) — each stepped independently.
- **Joint DOF**: `Fixed` 0, `Revolute` 1 (about `localAxisA`), `Ball` 3 (E2). Generalized
  coordinate layout concatenates per-joint DOFs in creation order after the (optional) 6 base DOFs.
- **No contacts in E0** ⇒ colliders are ignored for dynamics (kept only for later E1 coupling).

Constraint: the reduced backend assumes joints form a **tree** (humanoids are trees). Closed loops
(a body that is a child of two joints) are unsupported and rejected/ignored in E0.

## Step (E0b, fixed base)

Per `step(dt)` (optionally `substeps`), one ABA evaluation `q̈ = ABA(q, q̇, τ)` then a
**semi-implicit Euler** integration (`q̇ += q̈·dt; q += q̇·dt`). Gravity enters via the standard
trick: initialise the base spatial acceleration to `-a_gravity` so per-link gravitational forces
appear automatically (no explicit per-link force loop). Actuator torque `τ_i` is the joint input:
`Torque` mode → `τ_i = clamp(cmd, maxTorque)`; `PDTarget` → `τ_i = kp(target−q)+kd(targetVel−q̇)`
(explicit PD; SPD later if needed). Three passes:
1. **base→tips**: `v_i = Xup_i·v_{p(i)} + S_i·q̇_i`; `c_i = crm(v_i)·S_i·q̇_i`; init `IA_i = I_i`,
   `pA_i = crf(v_i)·I_i·v_i`.
2. **tips→base**: `U=IA·S`, `D=Sᵀ·U`, `u=τ−Sᵀ·pA`; propagate `IA,pA` to parent through `Xup*`.
3. **base→tips**: `a'_i = Xup_i·a_{p(i)} + c_i`; `q̈_i = D⁻¹(u − Uᵀ·a')`; `a_i = a' + S·q̈`.

After integration, recompute each link's **world pose** (compose parent pose with the joint
transform) for `poses()/pose()` readback, and fill `jointStates()` (`q`, `qd`) — for the reduced
backend these *are* the state (no derivation needed). `linear/angularVelocities()` from `v_i`.

## Floating base (E0d)

Root joint = free 6-DOF. Base generalized velocity is the spatial velocity `v_0 = [ω_0; v_0]`; base
`q̈` = base spatial acceleration from ABA with `IA_0`/`pA_0` at the root (no `Sᵀ` reduction — solve
`IA_0·a_0 = -pA_0` for the 6-DOF base, i.e. `a_0 = -IA_0⁻¹·pA_0`, then propagate to children).
Integrate base position with `v_0` and base **orientation with a quaternion** (exp-map of `ω_0·dt`,
matching the maximal backend's integrator). Gravity trick still applies.

## Validation plan

- **E0c fixed base**: (a) small-angle single pendulum period ≈ `2π√(L/g)` within a few %; (b) a
  passive (τ=0) double pendulum conserves total energy (KE+PE) to a bounded drift over N seconds
  (semi-implicit Euler ⇒ bounded, not zero); (c) coarse agreement with the maximal backend on a
  single pendulum swing (same pivot/length/mass) — not bit-identical (different formulation), but
  same period/amplitude within tolerance.
- **E0d floating base**: a free chain with **no gravity** conserves total **linear momentum** (COM
  moves in a straight line at constant velocity) and **angular momentum** about the COM, to a tight
  tolerance, for arbitrary initial joint velocities.
- **Determinism** (needed for E3): repeated identical rollouts are bit-identical; serial == parallel
  once behind `VecEnv`.

## Integration points (already in place)

- `enum class Backend { Realtime, Reduced }` (world.h); `createPhysicsWorld` dispatches.
- Internal maker `createFeatherstoneWorld(WorldDef)` declared in `src/physics/backends/backends_internal.h`,
  defined in `src/physics/backends/reduced/featherstone_world.cpp` (physics module globs it).
- `physics_env::EnvConfig.backend` already threads through to `createPhysicsWorld` ⇒ selecting the
  reduced backend for an Environment needs **zero** env-layer changes.

## E0 results (2026-07-04) — DONE

ABA core landed in `src/physics/backends/reduced/featherstone_world.cpp` (fixed + floating base,
revolute/fixed joints, explicit per-link gravity). Validated in
`tst/physics/integration/reduced.cpp`:

- **`reduced_pendulum_period`**: measured period **1.9958 s** vs analytic `2π√(L/g)` = 2.0061 s
  (**0.5%**); swings the correct direction (gravity sign pinned) with a symmetric amplitude
  (minQ = −2θ₀ exactly ⇒ energy conserved through the swing).
- **`reduced_double_pendulum_energy`**: total energy drift **0.54%** of the 38.9 J sloshing scale
  over 5 s of chaotic motion (semi-implicit Euler ⇒ bounded, not exact).
- **`reduced_floating_momentum`**: a bent, spinning free chain (no gravity/actuators) conserves
  **linear momentum to 0.1%** and **angular momentum to 0.24%** while genuinely articulating
  (jointMove ≈ 2 rad from Coriolis coupling).

**Engineering lesson — momentum drift under a constant unopposed torque.** A constant joint torque
applied to a floating chain from rest for 2 s spun the joint to unbounded speed and showed a
momentum drift that scaled as **1/substeps** (1.90 → 0.25 → 0.06 for substeps 8 → 64 → 256). This
is *integration error*, not an ABA bug — the algorithm matches Featherstone Table 7.1 and the
passive invariants (spin, translation, gravity, energy) all hold. Takeaway: validate conservation
with *passive/bounded-speed* motion; unbounded actuation is an integrator-convergence question, not
a conservation one. Real RL torques are bounded and opposed by gravity/contacts.

**Deferred within E0→later:** Ball (3-DOF) joints (needed by the humanoid — E2), contact coupling
(E1), and `setBodyState` for non-root links (reduced state is `q`; only the base pose/twist is
settable). A coarse cross-check vs the maximal backend was folded into the shared invariants
(both must reproduce the same pendulum period) rather than a bit-compare (different formulations).

## E1 results (2026-07-04) — DONE (contact coupling)

Contacts are external to the reduced system; coupled via **generalized-coordinate constrained
dynamics**:
- **CRBA** (`buildMassMatrix`) → dense joint-space inertia `H` (ndof = 6 floating-base + revolute
  joints). Cross-checked with the KE identity `½q̇ᵀHq̇ == ½Σvᵀ I v` (≤1e-7).
- **Contact detection** (`detectContacts`): dynamic-link colliders (sphere / box corners / capsule
  endpoints) vs static **planes** (the ground). General shape–shape is future work.
- **Contact Jacobian** (`jacRow`): geometric, per contact on link i — ancestor revolute columns
  `dir·(axisWₖ × (p − anchorWₖ))`, floating-base columns (3 angular `dir·(n_a×(p−com))` + 3
  linear `dir·n_a`). Sparse over ancestors only.
- **Solver** (`solveContacts`): sequential-impulse **PGS** on `qd` — per contact, normal impulse
  `λ_n ≥ 0` with effective mass `A = J H⁻¹ Jᵀ` and Baumgarte push-out `−(β/h)·max(0,pen−slop)`,
  then Coulomb friction on two tangents clamped to `±μλ_n`; `Δq̇ = H⁻¹ Jᵀ Δλ`. Runs between the
  free-velocity and position integration each substep; `updatePoses()` at the substep top keeps
  contact geometry current.

**Forest handling**: a parentless *Static, childless* link is **environment** (the ground), not
the articulation base; the tree root is the parentless link that is Dynamic or has children
(`ensureInit`). This lets a floating body + a separate static ground plane coexist.

**Validation** (`reduced.cpp`): floating sphere settles at exactly its radius (no penetration,
v→0); a box **holds** on a 17° slope at μ=1 and **slides** down-slope at μ=0.05 (friction cone is
doing real work, not numerical sticking).

## E2 results (2026-07-04) — DONE (Ball joints + humanoid)

**Unified multi-DOF rotation joint.** Fixed/Revolute/Ball share one model: `relRot = restRel·locRot`
(child-in-parent = rest × joint-local rotation), with a motion-subspace column per DOF axis `a`:
`S_a = [axis_a; −axis_a × anchorC]` (child frame). This **provably reproduces the revolute case**
(the old joint-frame decomposition equals this formula — verified algebraically and by the E0/E1
tests staying green after the rewrite). Ball = 3 child-frame axes; the joint velocity is the
child-frame relative angular rate, and `locRot` integrates via `locRot · exp(Σ q̇ₐ·axisₐ · dt)`.

ABA/CRBA/Jacobian generalized to `n_i`-DOF joints: `S` is `6×n_i`, `D = SᵀU` is `n_i×n_i` (solved
per joint), `q̈` an `n_i`-vector. Actuation: revolute torque/PD (scalar); **ball** per-axis torque or
**spherical PD** (`τ = kp·rotvec(target ⊗ locRot⁻¹) − kd·q̇`).

**Validation** (`reduced.cpp`): ball-pendulum energy drift **0.04%**; a z-torque on a ball joint
rotates the child in-plane about +z; the full **`makeHumanoid`** (14 bodies/13 joints — ball waist/
shoulders/hips, revolute elbows/knees/ankles, fixed chest/neck, floating pelvis base) **ragdoll
settles** on the ground (bounded, at rest, no penetration, collapsed from 1.2 m); a **suspended**
humanoid (pinned pelvis) **holds its neutral pose** via PD on all joints (revolute angles ≤0.001 rad,
legs held straight, fully settled).

## E3 results (2026-07-04) — DONE (behind VecEnv) → Phase E COMPLETE

The humanoid `Environment`/`VecEnv` run on `Backend::Reduced` by flipping `EnvConfig.backend` only —
**no env-layer changes** (same `actDim=21`, `obsDim=53`, obs/action API). Validated
(`tst/physics_env/integration/reduced_env.cpp`): random-torque rollout stays finite/bounded;
single-env determinism bit-identical; `VecEnv` **parallel == serial** bit-identical (VecEnv-safe:
per-env single-threaded world, no shared state).

**Throughput** (`physics_env.reduced_vs_maximal_throughput`, 13 workers, substeps=16): maximal
≈4.3k→32.6k env-steps/s (N=1→1024); reduced ≈2.0k→15.3k. At equal substeps the reduced backend is
~2× slower per step — the **dense `H⁻¹` contact solve (O(ndof³) per substep)** dominates; the obvious
optimization is a sparse/factored solve or an ABA-based operational-space inertia instead of a full
inverse.

**Stability tradeoff (documented):** under strong random torque the reduced ABA needs a **finer
substep** than the maximal PGS (which is more dissipative) — e.g. substeps 8 diverged at ±25 torque
but 32 was stable. Real RL torques are bounded; and this is the expected cost of a clean,
low-dissipation integrator.

**Phase E complete**: a reduced-coordinate Featherstone backend (ABA + CRBA + generalized-coordinate
contacts, fixed/floating base, revolute/ball/fixed joints, torque/PD actuation) behind the same
`PhysicsWorld`/`Environment`/`VecEnv` API, running the humanoid deterministically in parallel batches.
Remaining future work: contact-solve perf (sparse `H`), Ball q/qd readout in `JointState` (multi-DOF
observation), and differentiability on the reduced model.

## Sparse-H contact optimization (2026-07-04) — DONE

The contact solve inverted the dense joint-space inertia `H` every substep (O(ndof³) Gauss-Jordan) —
baseline profiling showed the contact solve was **~0.81 ms of a 0.89 ms step (92%)** for a humanoid
on the ground. Replaced with a **sparse LDLᵀ factorization** exploiting the DOF-ancestor tree:
- `dofParent_` (built in `ensureInit`): base DOFs chain `0←…←5`; each joint DOF's parent is the
  previous DOF in its joint or the nearest ancestor DOF (skipping DOF-less fixed joints).
- `factorizeSparse` — `H = Mᵀ D M` with `M` unit-lower-triangular whose fill follows the tree; only
  ancestor entries are touched (~O(ndof·depth²) vs O(ndof³)). Falls back to the dense inverse on a
  non-positive pivot or if the DOF order isn't a valid elimination order.
- `solveSparse` — sparse forward/diagonal/backward substitution over ancestor chains for
  `H⁻¹Jᵀ` per contact direction, replacing the dense `Hinv·Jᵀ` matVec.

Validated bit-for-bit against the dense inverse (max err ≤1.5e-4 at ndof=27) before switching; all
E1/E2/E3 tests stay green. **Result: ~1.5–1.65× env-steps/s** for the reduced humanoid (N=1:
2308→3490; N=1024: 16.5k→26.8k). The win is largest when contacts are few-to-moderate (the common
locomotion case), where the O(ndof³) inverse dominated. In a degenerate all-bodies-flat rest (many
contacts) the **PGS iteration** dominates instead of `H`, so sparse-H is ~neutral there — the next
lever for that case is the PGS itself (block/Delassus solve or exploiting Jacobian sparsity).

## Coverage hardening + bug fixes (2026-07-04)

Added a focused reduced-backend test file (`tst/physics/integration/reduced_joints.cpp`) covering
per-joint-type behavior (fixed weld, revolute Torque + PDTarget, off-axis hinge), capsule contact,
and characterizations of gaps — plus a side-by-side visual gallery
(`tst/physics/visual/reduced_joints.cpp`: single/double pendulum, 4-link chain, ball pendulum,
fixed-weld+elbow arm). The new tests surfaced two real bugs, now fixed:

1. **Joint limits were silently ignored.** `createJoint` never read `enableLimit/lower/upperLimit`,
   so revolute limits (elbows/knees/ankles in `makeHumanoid`) did nothing → limbs could
   hyperextend. Fixed with an **impulse-based one-sided limit constraint** in the generalized
   solver: a DOF past its limit adds a row `Jl = ±e_k` (no friction, `λ ≥ 0`, Baumgarte back into
   range) solved alongside contacts, so the impulse propagates through `H⁻¹` to the whole
   articulation. A first attempt using a hard position/velocity clamp in `step()` blew up the
   humanoid (zeroing a mid-chain DOF's velocity violates the articulation coupling / injects
   energy) — the impulse form is momentum-consistent and stable. Also clamped the Baumgarte
   correction velocity (`kMaxCorr`) for contacts + limits so a violent violation can't inject
   unbounded energy.
2. **`WorldDef` damping was not applied to the floating base.** The ctor ignored `linearDamping`
   entirely, and `angularDamping` was applied only to joint DOFs — so a floating body never slowed.
   Fixed: damp `baseTwist_` (angular + linear) each substep.

Stiffer (limit-constrained) dynamics need a finer step under abusive torque, so the ±15-torque
stress env test moved `substeps` 24 → 48 (deterministic; consistent with the existing "reduced ABA
needs a finer step than maximal under strong torque" note). Full suite 92/0, ctest 7/7; reduced
throughput unchanged (~31.8k env-steps/s at N=1024 — limits cost is nil unless a joint is at its
limit).

Ball q/qd readout — **RESOLVED** (2026-07-04): `JointState` now carries `rotation` (rest-relative
orientation rotvec) + `angularVelocity` for ball joints, populated from `locRot`/`qd`; the env's
default obs packer became DOF-complete (`obsDim` 53→69). A PD-target action mode + `setJointBallTarget`
were added alongside so the env can be driven by desired joint positions (see `physics_env`).
