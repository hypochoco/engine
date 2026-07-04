# Differentiable reduced-coordinate environment — design review

Dated design review (2026-07-04). Fleshes out the long-standing backlog item *"differentiability /
analytic gradients on top of the reduced model."* **This is a review + recommendation, not an
implementation.** It decides *whether*, *what*, and *in what order* before any code changes.

Grounds against the current forward pipeline in
`src/physics/backends/reduced/featherstone_world.cpp` and `src/physics_env/environment.cpp`.

## 1. Goal & what it buys us

Make the environment step **differentiable**: expose gradients of the next state (and of the
downstream observation) with respect to the current state and action. Concretely, for one control
step `s_{t+1} = f(s_t, a_t)` we want the Jacobian blocks

```
∂s_{t+1}/∂s_t   (state transition)      ∂s_{t+1}/∂a_t   (action sensitivity)
```

and the ability to chain them over a horizon (backprop-through-time).

Why it matters — it unlocks a class of methods that are far more sample-efficient than the
model-free PPO baseline:
- **Analytic policy gradients** (SHAC, short-horizon actor-critic, PODS): differentiate a
  short rollout's return w.r.t. policy parameters directly instead of estimating it from samples.
- **Differentiable MPC / trajectory optimization**: gradient-based control.
- **System identification / domain tuning**: fit sim parameters (mass, friction, gains) to data.
- **First-order model-based RL**.

Explicitly **out of scope / non-goals**:
- It does **not** gate walking training. PPO/SAC over `VecEnv` need *zero* differentiability. This
  is a parallel, higher-effort research track (see §9).
- Second-order (Hessians) — note where it'd be needed but don't build for it.
- Differentiating the **maximal** backend — its iterative LCP/PGS over redundant coordinates is the
  wrong substrate (see §3); the reduced model is what real differentiable sims use.

## 2. State manifold (the first subtlety)

The reduced state is **not** a flat vector. It is
`s = (basePos ∈ ℝ³, baseQuat ∈ SO(3), baseTwist ∈ ℝ⁶, {jointᵢ: locRot∈SO(3) or qᵢ∈ℝ^{dof}, q̇ᵢ})`.
Orientation lives on **SO(3)**, so gradients must be taken in the **tangent space** (the Lie
algebra 𝔰𝔬(3)) — a 3-vector per rotation, not 4 quaternion components. Consequences:
- The "generalized position" for gradient purposes is `(base translation ∈ ℝ³, base rotvec ∈ ℝ³,
  joint angles)`; velocity is already a proper tangent vector (`baseTwist`, `q̇`).
- Any Jacobian we expose should be **in tangent coordinates** (dim = ndof for both position and
  velocity blocks), matching the generalized-velocity dimension the backend already uses.
- Quaternion **normalization** (`glm::normalize` after every integrate) and the `w<0` sign flip in
  `quatToRotvec` are manifold housekeeping — differentiable in the interior but need care so
  gradients don't leak through the normalization/sign branch.

This is exactly why the reduced backend is the right substrate: `ndof` tangent coordinates, no
redundant constraints to differentiate.

## 3. Differentiability of each pipeline stage (grounded in our code)

| Stage (function) | Smooth? | Notes for AD/adjoint |
|---|---|---|
| ABA `computeAccelerations` | ✅ smooth | Matrix/cross products, 6×6 solves, per-joint `dof×dof` inverse. Differentiable; the `solve6`/`invertDense` **partial pivoting** (`piv`, `std::swap`) is a data-dependent branch — for gradients use a **fixed** factorization (no pivot reorder) or LDLᵀ. |
| CRBA `buildMassMatrix` | ✅ smooth | Pure spatial-inertia accumulation. |
| Sparse LDLᵀ `factorizeSparse`/`solveSparse` | ✅ smooth (fixed pattern) | The `dofParent_` tree is fixed at init, so the sparsity pattern is constant ⇒ differentiable. The pivot guard `if H[k][k] < 1e-12 return false` + dense fallback is a branch, but off the degenerate path it's smooth. |
| Joint kinematics `jointRel`/`relRot`/`Xup` | ✅ smooth | Quaternion→matrix, rest-relative compose. |
| Integration `quatFromRotvec`/`integrateQuat`, `q += q̇·h` | ✅ smooth | Small-angle guards (`< 1e-9`) + `atan2` are smooth in the interior; guard branches need a consistent subgradient at 0. |
| Actuators `jointTorques` (Torque, PD) | ✅ smooth | Linear in `q, q̇, target`; ball PD uses `quatToRotvec` of a quaternion error (smooth away from π). `maxTorque` **clamp** is a kink (subgradient 0 when saturated). |
| **Contact detection** `detectContacts` | ❌ **discontinuous** | The *active set* changes discretely as bodies touch/separate — the fundamental non-smoothness. While a contact is active the plane point/normal/penetration are smooth. |
| **Manifold reduction** (sort + cap-4) | ❌ discrete | Selects which contacts survive — a `stop-gradient` selection; the kept quantities are smooth. |
| **Contact PGS** (12 iters) | ❌ **non-smooth + iterative** | Per-iteration kinks: normal `max(0, ·)` (complementarity), **circular friction-cone projection** (`mag > μλn`), Baumgarte `max(0, pen−slop)`, and the `kMaxCorr` clamp. Warm-start cache is a `stop-gradient` read. |

**Takeaway:** ~everything except contact is smooth and differentiable with modest effort. **Contact
is the whole problem.** For locomotion the useful gradient signal (PD control → joint/base motion)
flows mostly through the smooth path; contact contributes both essential ground-reaction gradients
*and* the hard discontinuities.

## 4. Approaches to differentiable contact (the core decision)

- **(A) Smoothed / compliant contact.** Replace the hard LCP with soft penalty springs + smoothed
  nonlinearities (softplus for `max(0,·)`, smooth friction saturation). *Pro:* trivially
  differentiable, no active-set problem, matches Brax "spring"/"positional" and DiffTaichi. *Con:*
  biases the physics (soft ground), needs stiffness tuning, stiffer ⇒ smaller stable step (we saw
  this with the hard limits → substeps 24→48). A **separate differentiable contact model**, not our
  production PGS.
- **(B) Implicit differentiation (IFT through the LCP/QP).** Differentiate the *converged* contact
  solution via the implicit function theorem on the KKT/complementarity conditions — the active set
  is treated as locally fixed, and gradients come from one linear solve with the active constraint
  Jacobian (OptNet/Dojo/DiffCoSim). *Pro:* exact gradients at the solution without unrolling; reuses
  our `H` factorization. *Con:* more math; needs the active/inactive/sticking-sliding partition at
  the solution; still discontinuous *across* active-set changes (inherent).
- **(C) Unroll the PGS and autodiff the iterations.** *Pro:* mechanical with an AD tape. *Con:*
  gradients of a *non-converged* iterate, subgradients through every clamp, and memory ∝ iterations
  (12) × contacts — plus vanishing/exploding gradients. Generally the worst option for our
  sequential-impulse solver.
- **(D) Randomized smoothing / zeroth-order for contact only.** Smooth the discontinuity by
  expectation (MuJoCo-MJX-style analytic + sampling), or let a learned critic absorb contact-mode
  switches (SHAC's stance). Complements (A)/(B).

**Recommendation:** **hybrid**. Analytic/AD through the smooth dynamics (highest value, lowest risk),
and for contact **start with (A)** a smoothed compliant model *behind a flag*, keeping the hard PGS
as the default forward-only solver. Revisit **(B) IFT** once (A) validates the plumbing and if the
soft-contact bias hurts. Accept that the active-set discontinuity is fundamental — short horizons +
a critic (SHAC) are how the field lives with it.

## 5. AD strategy (how we actually get derivatives)

- **Hand-written adjoints.** Fastest runtime, but the most work and brittle — and it fights our
  hand-tuned `factorizeSparse`/`solveSparse`/warm-start/block-friction (each needs a matching
  adjoint). Rejected as the *starting* point.
- **Templating the dynamics on a `Scalar` type** (instantiate for `double` and for an AD dual/var
  type). Least *conceptually* invasive: write the math once. **Blocker:** the backend leans on
  **glm** (`Vec3/Quat/Mat3` are float/double-specific), so this needs either a minimal
  `Scalar`-generic linalg (vec3/quat/mat3/6×6) or adopting **Eigen + a small autodiff header**
  (e.g. `autodiff`/CppAD/`Enzyme`). A real refactor of the hot path.
- **Reverse-mode tape over the step.** Most flexible, natural for BPTT (scalar loss, many inputs).
  Heaviest to build/integrate and hardest to keep fast.
- **Forward-mode duals.** Cost ∝ #inputs; our input dim (state ndof + action ~ 27+21) is large, so
  full Jacobians via forward mode are expensive — but fine for *directional* derivatives / JVPs.

**Recommendation:** do **not** template the existing optimized TU in place (risks the forward perf we
just earned). Instead write a **separate, AD-friendly differentiable step** that **shares the model
data** (`Link`/`Joint`/tree) but re-implements the dynamics over a `Scalar`-generic minimal linalg,
using **reverse-mode AD** (VJP-oriented, for BPTT). The two step implementations are cross-checked
against each other (value equality) and against **finite differences** (gradients). This keeps the
fast path pristine and isolates the AD complexity. If duplication proves too costly, promote the
shared math to a `Scalar` template later.

## 6. Interaction with the forward-path optimizations

- **Sparse LDLᵀ `H`**: differentiable given the fixed `dofParent_` pattern; its adjoint is a second
  sparse solve — reusable. Good.
- **Warm-starting** (`impulseCache_`): a `stop-gradient` — gradients must not flow through the cached
  seed (it's an optimization, not part of the mathematical map). Easy to honor.
- **Manifold reduction / active-set**: `stop-gradient` on the *selection*; differentiate only the
  kept, active contacts.
- **Block friction + `kMaxCorr` clamp**: kinks; smoothed in the differentiable contact model (A).
- Net: the **differentiable path likely uses a simpler contact solver** (smoothed, fixed active set)
  than the production PGS — that's expected and fine (Brax does the same: separate solvers).

## 7. API sketch (kept behind the existing boundary)

Options, cleanest first:
1. **VJP hooks at the env boundary.** The differentiable step returns, alongside `s_{t+1}`, a
   callable/linear operator for `vⱼ ↦ (∂s_{t+1}/∂(s_t,a_t))ᵀ vⱼ`. A downstream Python autodiff
   framework (JAX `custom_vjp` / PyTorch `autograd.Function`) wraps it. Minimal surface, matches how
   these get consumed.
2. **Dense Jacobian output**: `step` optionally fills `∂s_{t+1}/∂s_t (ndofₛ×ndofₛ)` and
   `∂s_{t+1}/∂a_t (ndofₛ×actDim)` in **tangent coordinates**. Simple, fine at humanoid scale.
3. A `DiffEnvironment` sibling to `Environment` (same config/obs/action), so forward-only users pay
   nothing. Reset is a **stop-gradient boundary** (episode starts detach the graph).

Keep it backend-agnostic in spirit, but **only the reduced backend implements it** (the maximal one
returns "not differentiable").

## 8. Validation methodology

- **Value parity**: the differentiable step must reproduce the production step within tolerance
  (same model, same math) — a regression guard.
- **Gradient check vs finite differences**: central differences on the manifold (perturb in tangent
  space, re-project), per input component, on scenarios of increasing hardness:
  1. single pendulum, `∂(final angle)/∂torque` (smooth, no contact) — must match FD to ~1e-4.
  2. double pendulum / free-floating chain `∂/∂(state,action)`.
  3. sphere-on-plane `∂(rest height)/∂(drop height, friction)` — exercises **contact** gradients.
  4. one-leg hopper `∂(hop distance)/∂(PD targets)` — the locomotion-relevant case.
- **Determinism** (already 0.0 serial↔parallel) is a prerequisite and stays a gate.

## 9. Recommendation & phasing (slip-able; does NOT gate PPO)

Treat as **Phase F** — a separate track from the walking-training bring-up.
- **F0** — *this review* + pick: contact = smoothed-(A)-first, AD = separate reverse-mode step. ✅ (this doc)
- **F1 — differentiable smooth dynamics (no contact).** AD step for ABA/CRBA/joints/actuators;
  gradient-check on pendulum + free chain. **High standalone value** (in-air control, system ID).
- **F2 — differentiable contact (smoothed model).** Add compliant contact behind the diff path;
  gradient-check sphere-on-plane + hopper; document the soft-contact bias vs the hard forward solver.
- **F3 — VJP at the env boundary** + a short-horizon analytic-gradient smoke test (SHAC-style,
  downstream) to prove end-to-end usefulness.
- **F4 (optional) — IFT/exact contact gradients** if (A)'s bias is limiting.

**Sequencing vs the walking goal:** PPO + Python bindings unblock walking *now* and need none of
this. Recommend: **bindings + a PPO baseline first**, differentiability as the Phase-F track after —
**unless** differentiable-sim methods (SHAC/analytic PG) are the primary research objective, in which
case F1–F3 leapfrog PPO.

## 10. Risks & open questions

- **Active-set discontinuity is fundamental** — no formulation removes it; smoothing/short-horizons
  only manage it. Set expectations accordingly.
- **glm dependency** in the hot path blocks straightforward templating ⇒ the separate-AD-step
  approach (or an Eigen/autodiff adoption) is the pragmatic dodge; cost is code duplication.
- **Perf**: the AD step is much slower than forward; it's for training/optimization, not the 30k
  env-steps/s throughput path. Keep them separate.
- **Memory** for BPTT over long horizons (checkpoint/truncate).
- **Soft-contact fidelity**: does (A)'s bias meaningfully change learned gaits vs the hard solver?
  (F2 must measure this.)
- **Second-order** methods (differentiable MPC with Newton) would need Hessians — out of scope now;
  the reverse-mode choice doesn't preclude it later.

## Decision

Proceed **only when differentiable-sim methods become a priority** (not before the PPO/bindings
bring-up). When we do: **separate reverse-mode AD step sharing the reduced model**, **smoothed
compliant contact** behind a flag, **VJP at the env boundary**, validated by finite differences —
phased F1→F3, with IFT-based exact contact gradients (F4) held in reserve.

---

## A vs D vs Hybrid — speed, training, and the refined plan (2026-07-04)

**Context update:** differentiable-sim methods are now the **primary research objective**, so this
section supersedes the §4/§9 recommendations and the Decision above.

### Reframe: (D) doesn't need a differentiable simulator
**(A)** is what actually *makes the sim differentiable* — the research substrate. **(D)** (randomized
smoothing / zeroth-order) only needs **fast forward evals**; it estimates the gradient of the
Gaussian-smoothed objective by sampling perturbed rollouts and works through a hard,
non-differentiable contact solver. So with differentiable-sim as the goal, pure-D undercuts the
premise — its role is the **baseline** and the **zeroth-order half of the hybrid**.

### Speed (compute per gradient)
| | (A) analytic/smoothed | (D) zeroth-order | Hybrid (α-order) |
|---|---|---|---|
| per-gradient | 1 fwd + 1 backward | N perturbed fwd rollouts | AD backward + adaptive sampling |
| backward | reverse/fwd AD, ~2–4× fwd + tape mem | none | some |
| step used | the **slow** differentiable step | the **fast** production step (~31k eps/s batched) | mostly slow |
| scales with | ~indep. of #params | #samples N ∝ dim/variance | between |
| parallelism | batch B | N·B evals — ideal for our ThreadPool | both |
| memory | O(H·state) tape | O(1) | O(H·state) |
| engine effort | **High** (build AD step + smoothed contact) | **Low** (reuses fwd path; logic downstream) | **Highest** (both) |

Grounding: forward reduced humanoid ≈ 4.1k eps/s (N=1), ≈31k batched; the AD step will be ~5–15×
slower/step (loses sparse-H/warm-start; smoothed-but-stiff contact wants more substeps — cf. limits
24→48) plus the backward. So **(A) is cheaper per-gradient**, **(D) is more FLOPs but on the fast
path and perfectly parallel** (zero new infra), **hybrid is priciest per-gradient but adaptive**.

### Training (gradient quality, sample efficiency, robustness)
| | (A) analytic/smoothed | (D) zeroth-order | Hybrid (α-order) |
|---|---|---|---|
| gradient bias | soft-contact bias; **can explode** through stiff contact | unbiased for smoothed objective | adaptively low |
| variance | low | higher (∝ dim/N) | balanced |
| robust to contact discontinuity | ❌ | ✅ | ✅ |
| sample eff. (benign) | **best** | moderate | best |
| sample eff. (contact-rich) | often poor/unstable | reliable | best |
| algorithms | SHAC, short-horizon BPTT, diff-MPC | ES, zeroth-order PG | α-order PG |
| contact fidelity / transfer | trains on **soft** sim (bias) | trains on **true hard** sim | mixed |

Key result — **Suh et al. 2022, "Do Differentiable Simulators Give Better Policy Gradients?"**:
first-order analytic gradients through stiff/discontinuous contact are frequently **biased and
high-variance** (chaotic landscape — a contact-mode flip → exploding gradient), while the
zeroth-order estimator is **unbiased for the smoothed objective** and often lower effective variance
there. Their **α-order estimator** (the hybrid) interpolates first-/zeroth-order per a bias–variance
criterion — cheap analytic where smooth, robust sampling where stiff. This is precisely the hybrid.

### Refined AD strategy
Per-step, **use forward-mode dual numbers to produce the state-transition Jacobian
∂s_{t+1}/∂(s_t, a_t)** in tangent coordinates, and expose it (or a VJP) at the env boundary;
**downstream frameworks chain the per-step Jacobians** (their own BPTT). At humanoid scale
(state≈2·ndof≈54, act=21) #inputs≈#outputs, so forward-mode is competitive and *far* simpler to
build/validate than an in-engine reverse-mode tape. Promote to reverse-mode only if long-horizon
in-engine BPTT becomes the bottleneck.

### Decision → build the hybrid, in this order
1. **(A) is the core** — build the differentiable step; **D as the cheap parallel baseline** (reuses
   the fast `VecEnv`, logic mostly downstream); **converge on the hybrid (α-order)** as the end state
   and the defensible research result.
2. **Phase F sequencing:**
   - **F1** — differentiable **smooth dynamics** (no contact) via forward-mode duals: `Dual` AD type
     + Scalar-generic linalg → generic ABA; gradient-check vs central finite differences (pendulum →
     chain → floating base + ball). Highest value, lowest risk.
     - **F1a/b DONE** (2026-07-04): `include/engine/physics/diff/dual.h` (forward-mode `Dual<N>`);
       differentiable 1-DOF pendulum gradient-checked vs FD to 8 digits + analytic period.
     - **F1c DONE** (2026-07-04): Scalar-generic ABA for fixed-base revolute/fixed chains
       (`diff/linalg.h` V3/M3/V6/M6 spatial algebra + Rodrigues, no quaternions; `diff/articulated.h`
       `DiffModel` + `diffForwardDynamics`/`diffSubstep`/`linkWorld`). Validated: single-pendulum
       period (2.0075 vs 2.0071), double-pendulum energy drift 0.08%, and ∂(final q)/∂(initial angle,
       torque) via `Dual` == central FD to 8 digits.
     - **F1d DONE** (2026-07-04): unified `DiffState<S>` (base pose/twist + per-link rotation matrix
       + joint qd) + generalized ABA for **fixed/floating base** and **revolute/ball/fixed** joints;
       SO(3) exp-map integration (base pose + joint rotations), 6×6 SPD base solve + 3×3 ball D
       inverse (`diff/linalg.h` `expSO3`/`solveM6`/`invertSmall`). Validated: revolute period +
       double-pendulum energy 0.08%, **ball**-pendulum energy 0.03% (swings to −L), **floating**
       free-chain linear/angular momentum drift 0.03%/0.06% with real articulation — and **all**
       gradients (revolute/ball/floating, w.r.t. angle/torque/base-twist) match central FD to 8
       digits. **Phase F1 (differentiable smooth dynamics) COMPLETE** — full humanoid joint/base
       coverage, quaternion-free.
     - **NEXT**: per-step state/observation Jacobian ∂s_{t+1}/∂(s_t,a_t) at the env boundary (tangent
       coordinates), then **F2** smoothed compliant contact.
   - **F2** — differentiable **smoothed/compliant contact** behind a flag; gradient-check
     sphere-on-plane + hopper; measure the soft-contact bias vs the hard forward solver.
     - **F2a DONE** (2026-07-04): smoothed compliant ground contact (plane y=0) folded into the
       generic ABA as an external force — normal force `k·softplus_β(pen) − c·vₙ·σ(β·pen)` (central,
       no torque), `softplus`/`sigmoid` added to `Dual`. Validated: a floating sphere rests at the
       compliant equilibrium `r − mg/k` (no tunneling, settles) and ∂(final height)/∂(initial
       height, velocity) **through active contact** matches central FD to 8 digits.
     - **F2b DONE** (2026-07-04): smooth **regularized Coulomb friction** (Ft = −μ·Fspring·slip/
       √(|slip|²+ε²), slip = the contact-point velocity v_com + ω×(−r·n)) + the **radius-lever
       contact torque** ⇒ a sphere **rolls without slipping** (validated: vx 2.0→1.435, ω_z=−7.17 =
       −vx/r exactly). Coupled **hopper** (floating base + revolute leg + foot contact): ∂(final base
       height)/∂(hip torque, initial height) matches central FD to 8 digits. Friction gradient
       matches FD too. **Soft-contact bias quantified**: the compliant plane rests at `r − mg/k`
       (bias 3.3 mm at k=3e3, tunable via k) vs the hard PGS solver's exact `r`. NB: a real physics
       bug was caught here — friction must oppose contact-point **slip**, not COM velocity, else it
       never vanishes at rolling.
     - **NEXT (F2 wrap)**: general link contact geometry (feet/multiple points) if needed; otherwise
       proceed to **Fd** (zeroth-order baseline) + **F3** (α-order hybrid + per-step Jacobian/VJP).
   - **Fd (parallel, cheap)** — **zeroth-order estimator** over the existing fast `VecEnv` as the
     baseline (mostly downstream; engine just serves batched perturbed rollouts deterministically).
     - **Fd DONE** (2026-07-04): `include/engine/physics/diff/zeroth_order.h` — deterministic
       antithetic Gaussian-smoothing (ES) gradient estimator `∇f_σ ≈ (1/N)Σ (f(x+σε)−f(x−σε))/(2σ)·ε`.
       Validated: unbiased on a quadratic (large-N est → analytic Ax+b, variance ↓ with N) and
       **agrees with the analytic `Dual` gradient to 0.69%** on the smooth pendulum objective — i.e.
       the two gradient sources the α-order hybrid blends coincide on smooth problems. The N
       evaluations are embarrassingly parallel → map onto the fast `VecEnv` downstream.
   - **F3** — **α-order hybrid**: blend the F1/F2 analytic gradient with the Fd zeroth-order estimate
     per a bias–variance criterion; expose per-step Jacobian/VJP at the env boundary; SHAC-style
     short-horizon smoke test downstream.
     - **F3a DONE** (2026-07-04): `include/engine/physics/diff/hybrid.h` — `alphaOrderGradient` blends
       per-sample first-order `∇f(x+σε)` (analytic) and zeroth-order `(f(x+σε)−f(x−σε))/(2σ)·ε` via the
       **minimum-variance convex weight** `α* = (VarB−Cov)/(VarA+VarB−2Cov)`. Validated (Suh et al.):
       on the SMOOTH pendulum α=1.000 and the blend equals the analytic gradient; on a STIFF near-step
       (β=200) α=0.15 and the α-order estimate has **4.5× lower across-seed variance** than pure
       first-order. The defensible "differentiable sims don't always give better gradients" result.
     - **F3b NEXT**: per-step state/observation Jacobian ∂s_{t+1}/∂(s_t,a_t) in tangent coordinates at
       the env boundary (forward-mode duals; tangent-space seeding for orientation), likely a
       `DiffEnvironment` mirroring `physics_env::Environment` exposing value + Jacobian/VJP for
       downstream BPTT; then a SHAC-style short-horizon smoke test.
     - **F3b (Jacobian) DONE** (2026-07-04): `include/engine/physics/diff/jacobian.h` — `stepJacobian`
       computes the per-step tangent-space Jacobian `∂s_{t+1}/∂(s_t,a_t)` column-by-column via exact
       forward-mode `Dual<1>` directional derivatives; orientation states seeded via the exp map and
       read out via `vee`/`log` (Lie-algebra tangent), so it's a plain dense matrix over ℝ^nState.
       Layout: `[baseRot 3][baseTrans 3][jointCfg ndof] ‖ [baseTwist 6][jointVel ndof]` (base blocks
       only if floating). Validated: the full 14×15 Jacobian of a floating-base+revolute step matches
       tangent-space central FD to **4e-11** (every block: base rot/trans/twist, joint cfg/vel, action).
     - **F3c DONE** (2026-07-04): `include/engine/physics/diff/from_articulation.h`
       (`articulationToDiffModel` — inertia/parent/DOF/anchors/restRel/axis + floating root + foot
       contact spheres, mirroring the reduced backend) + `diff_environment.h` (`DiffEnvironment`
       mirroring `physics_env::Environment`: `reset`/`setAction`/`step`, `jacobian()` = per-step
       tangent Jacobian, `rolloutGradient<NA>()` = analytic gradient of a scalar objective over a
       rollout). Validated on the **real humanoid**: converter reproduces authored poses (pelvis
       0.990, foot 0.030), 14 links / 21 DOF / floating; a random-torque rollout is finite+bounded;
       the 54×75 per-step Jacobian is finite; and the analytic gradient of a scalar rollout objective
       matches central FD to **1.3e-9**.
     - **Only remaining (downstream)**: a SHAC-style short-horizon analytic-policy-gradient smoke test
       lives in the downstream RL repo (reward/policy/optimizer), consuming `DiffEnvironment` +
       `alphaOrderGradient`.

**Phase F (differentiable reduced env — hybrid α-order) is COMPLETE on the engine side**: smooth
dynamics + contact are differentiable and FD-validated (to 8–11 digits), a zeroth-order baseline and
the α-order hybrid are built and validated, per-step tangent Jacobians are exposed, and the real
humanoid runs differentiably through a `DiffEnvironment`. All header-only under
`include/engine/physics/diff/` (`dual, linalg, articulated, zeroth_order, hybrid, jacobian,
from_articulation, diff_environment`), quaternion-free, every gradient checked.
   - **F4 (reserve)** — IFT/exact contact gradients if the smoothed-contact bias limits results.
