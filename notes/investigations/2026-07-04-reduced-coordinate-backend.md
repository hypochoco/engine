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

## E2 plan (next) — humanoid + Ball joints
The humanoid needs **Ball (3-DOF) joints** in the reduced backend (currently only revolute/fixed).
A ball joint's motion subspace is 3 angular DOFs (`S` = 3 columns); ABA/CRBA/Jacobian all generalize
to multi-DOF joints (D becomes 3×3, solved per joint). Orientation coordinate: integrate the joint's
relative quaternion from its 3 angular rates. Then `makeHumanoid` can build in the reduced backend;
targets: ragdoll settles on the ground, PD-stand holds a pose — mirroring the maximal-backend tests.
