# Articulation approach — maximal-coordinate constraints vs reduced-coordinate (Featherstone)

Design/decision note (2026-07-03). Point-in-time; the living summary is the Milestone 2 entries
in `notes/core/{goals,todo}.md` and the milestone plan
([2026-07-03-humanoid-rl-milestone-plan.md](2026-07-03-humanoid-rl-milestone-plan.md)).

## Status: DECIDED to DEFER the reduced-coordinate work

- **Now (Milestone 2, Phase B):** build **maximal-coordinate joint constraints + actuators** on
  the existing sequential-impulse solver. Fastest path to a visible, controllable, rendered
  humanoid; reuses everything we already have (solver, contacts, friction, deterministic
  parallel stepping). Unblocks input/terrain/env-interface against a real articulated body.
- **Later (integral once we actually train):** add a **reduced-coordinate (Featherstone/ABA)
  `PhysicsWorld` backend**. Not needed to *demonstrate* a walking humanoid, but it becomes
  important for *training throughput and stability* (see "Why this is integral at training
  time"). Our runtime-virtual multi-backend `PhysicsWorld` was designed for exactly this
  swap-in.
- **Guiding rule:** keep the joint/actuator + env/observation API **articulation-backend-
  agnostic** (reads joint `q/qd`, writes actuator torque/target) so switching backends is
  transparent to downstream training code and we don't rebuild the humanoid twice.

## The core difference: how the humanoid's state is represented

**Maximal coordinates (our solver today).** Every limb is a free 6-DOF rigid body; a 15-body
humanoid is 15×6 = 90 DOF in the representation. Joints don't remove DOF — each joint is a set
of **constraint equations** the solver keeps satisfied (hinge = 5 constraints, ball = 3).
Contacts are *also* constraints, so joints + foot-ground contacts share one solver.

**Reduced / generalized coordinates (Featherstone).** Represent the humanoid by its *actual*
DOF: root 6-DOF + one number per joint DOF (knee = 1 angle, hip = 3). That humanoid becomes
~23 generalized coordinates `q` with velocities `q̇`. Joints are baked into the parametrization,
so a knee **cannot drift off its axis** — there is no off-axis motion in the representation to
drift into.

## How each steps

**Constraints (PGS / sequential impulse):** each frame, apply impulses to drive constrained
relative velocities to zero + a Baumgarte/soft term for drift. Joints and contacts in one loop.
Small addition to what we have.

**Featherstone (Articulated-Body Algorithm):** solve "given `q, q̇, τ`, find `q̈`" in **O(n)**
via three sweeps over the limb tree using spatial algebra (6-vectors bundling linear+angular):
base→leaves (velocities/Coriolis), leaves→base (accumulate "articulated-body inertia" = how much
a joint resists acceleration given its free-moving children), base→leaves (solve joint
accelerations). Joint torque `τ` is a **direct input** — exactly the RL action.

## Comparison

| Dimension | Maximal + constraints | Reduced + Featherstone |
|---|---|---|
| Joint accuracy | approximate, can drift/soften | **exact by construction**, no drift |
| Stability (long chains / high mass ratio) | many iterations, can get spongy | **rock solid** |
| RL action space | torques on bodies; joint coords derived | joint coords *are* the state, `τ` direct — **ideal** |
| Observation size | tends toward full body poses (large, redundant) | minimal joint `q/qd` (**small, non-redundant**) |
| Per-step cost | varies with constraint difficulty | consistent **O(n)**, few iterations |
| Contacts | **unified & easy** (same solver) | **separate; coupling is the hard part** |
| Closed loops | natural | need extra loop constraints (humanoids are trees → N/A) |
| Differentiability | possible but messy through iterative solver | **cleaner** analytic gradients |
| Implementation cost | **small** (extends our solver) | **large** (new dynamics core) |
| Reuses our code | solver, contacts, friction, threading, determinism | only collision/broadphase |

Modern soft/TGS constraint solvers with substepping (Box2D v3, Rapier, PhysX-TGS) narrow the
stability gap a lot vs naive Baumgarte PGS — constraint-based characters are perfectly usable
for the *demonstration* milestone.

## Why reduced coordinates become integral at training time

Beyond stability, the decisive reason for training is **throughput**, and observation size is a
first-order lever:

- **Smaller, non-redundant observations.** In maximal coordinates the natural state is per-body
  pose+velocity (~13 numbers × ~15 bodies ≈ ~195), much of it redundant (a limb's world pose is
  implied by its parent + joint angle). Reduced coordinates give the *minimal* state — joint
  `q/qd` + root (~50-ish) — directly. Smaller observation vectors mean **smaller policy/value
  network inputs**, less data moved per step, and often **faster/*better* learning** (less
  redundant, better-conditioned state). Observation/throughput is a significant training
  bottleneck, so this compounds across millions of steps × thousands of envs.
- **Predictable O(n) step cost** with few iterations → steady, high env throughput when
  vectorized (fits our parallel-worlds path).
- **Drift-free stability** through the random-action early-training phase.
- **Cleaner gradients** if we later pursue differentiable/analytic-gradient methods (below).

So: fine to defer while we build features and *demonstrate* the humanoid; expect to invest in
the reduced-coordinate backend before serious training begins.

## The catch (why it's a big build): contacts

Featherstone gives clean *internal* dynamics, but contacts are **external** to the reduced
system and must be coupled back — compute contact-space inertia from ABA quantities and solve an
LCP/PGS in contact space (Bullet/PhysX style), or MuJoCo's soft-constraint convex formulation.
This coupling, plus spatial algebra and per-joint motion subspaces, is the bulk of the work. Our
existing collision/broadphase substrate is reused; the dynamics + contact coupling is new.

## Backend-agnostic joint/actuator API (build once, in Phase B)

To avoid rebuilding the humanoid when the second backend lands, Phase B should define these at
the `PhysicsWorld` boundary (both backends implement them):

- **Joint types:** `Ball` (3-DOF), `Revolute/Hinge` (1-DOF + limits), `Fixed/Weld`; frames on
  each body; angular limits; (later) cone/twist.
- **Actuator:** per joint DOF — mode ∈ {`Torque`, `PDTarget(targetPos, targetVel, kp, kd)`},
  torque limit. This is the RL action write-path.
- **State read-path:** joint `q` / `q̇` per DOF, plus root pose/twist and contact flags — the
  minimal observation. In maximal coords these are *derived* from body poses; in reduced coords
  they *are* the state. Same accessor either way.
- **Articulation builder + humanoid preset:** data description (bodies/joints/actuators) →
  instantiated into a `PhysicsWorld`; URDF/MJCF import later (likely downstream).

## Solver engineering notes (learned building B1, 2026-07-03)

Two non-obvious lessons from implementing maximal-coordinate joints on the sequential-impulse
solver — both matter for B2–B5 and for anyone touching the joint solve later:

1. **Solve the angular part before the point-to-point part, every iteration.** A joint's angular
   impulses change a body's angular velocity, which in turn perturbs the *anchor* velocity
   (`v_anchor = v_com + ω × r`). If the linear (point-to-point) constraint is solved first and the
   angular second, each iteration leaves the anchor freshly violated by the angular correction,
   and the coupled solve converges poorly or diverges. Solving angular first and the point
   constraint **last** means the anchor (linear) constraint is satisfied at the end of each
   iteration → far more stable. (This is why `solveJoints()` does angular → point.)

2. **Divergence in a stiff joint is usually an inertia-ratio problem, not a constraint bug.** The
   first `Fixed`-joint cantilever test exploded to ~1e4 in 240 steps. Root cause was **not** the
   constraint math — it was a pathological collider: a 0.1 m sphere has `invInertia = 1/(0.4·m·r²)
   = 250`, so the point impulse's induced spin `IinvB·(rB × P)` is amplified ~250× on a 1 m lever,
   destabilizing the coupled linear/angular solve. A realistically-sized body (r = 0.5 →
   `invInertia = 10`) holds the same cantilever rock-solid. Takeaway: extreme mass/inertia ratios
   (point masses on long levers) need more iterations/substeps, exactly like stiff contact stacks;
   real humanoid limbs (boxes/capsules whose size ≈ their lever) are well within the stable regime.
   When a joint test blows up, check the inertia/lever ratio before suspecting the solver.

3. **Explicit PD on a lightly-loaded compliant chain can be unstable at aggressive targets
   (learned building B5).** Driving the humanoid's *knees* to a bent target held cleanly and
   symmetrically, but driving the *elbows* (a light forearm hanging off a PD-compliant ball
   shoulder — a soft double pendulum) to a large bent target overshot and went L/R-asymmetric
   (2.11 vs 1.71 rad): explicit PD adds energy when the plant is soft and under-damped. Mitigations
   we already have: higher `kd`, more substeps, or stiffer parent joints; the real fix is
   **stable PD (SPD)** — an implicit PD that stays stable at high gains — which is the natural B-phase
   follow-up if precise high-gain pose tracking is needed. For now: gravity-stable targets and
   moderate gains hold reliably (the PD-stand test commands bent knees against gravity + neutral
   elbows).

## Differentiable physics — how it plays into ML here

(See also the deferred Phase 3 in the physics plan.) Differentiable physics means the simulator
can return **gradients of outputs w.r.t. inputs** (∂(next state, or a loss) / ∂(actions,
params, initial state)), not just the next state. Where it helps:

- **Analytic-gradient policy learning.** Instead of model-free RL (PPO/SAC) estimating a policy
  gradient from noisy sampled returns, you can backprop a differentiable loss *through the
  rollout* into the policy — far lower-variance gradients, often dramatically fewer samples for
  smooth locomotion/control (the Brax / MuJoCo-MJX / DiffTaichi line of work).
- **System identification / calibration.** Fit physical parameters (mass, friction, motor gains)
  to reference/mocap data by gradient descent through the sim.
- **Trajectory optimization & motion tracking.** Directly optimize control sequences to match a
  target motion (imitation) via gradients.
- **Differentiable rendering + physics** (further out): gradients from pixels back through
  dynamics for vision-in-the-loop tasks.

Big caveats: **contacts are non-smooth** (impulsive, stiff), so naive gradients through contact
are noisy/biased — practical differentiable sims use smoothed/soft contact models or randomized
smoothing, and it's an active research area. And **reduced coordinates make differentiability
much cleaner** (smooth, minimal state; analytic dynamics), which is another reason the
Featherstone backend and the differentiable backend are natural to pursue together.

For our roadmap: model-free RL (PPO/SAC) on the (eventually reduced-coordinate) sim is the
baseline path and needs **no** differentiability. Differentiable physics is an *accelerator/
research* option layered on later — most valuable once we have the reduced-coordinate backend,
and it targets sample-efficiency and sysid rather than being required for a walking policy.
