# Differentiable physics — semi-implicit contact testing round (findings)

**Date:** 2026-07-04
**Scope:** deep-dive testing of the semi-implicit contact integration (Feature 4) added to stabilize
the humanoid, and the compliant ground-contact **force model** it drives
(`include/engine/physics/diff/articulated.h`, `from_articulation.h`).
**Status:** investigation + tests complete, tree green (**141 tests, 0 failed; ctest 7/7**).
**Fixes are DEFERRED** per the testing-round plan — this report is the evidence, not the fix.

---

## What was read
- `articulated.h` — `diffSubstep` (Explicit vs `SemiImplicit` predictor→corrector), `diffApplyAccel`
  / `diffIntegrateConfig`, `groundContactSpatial` (the compliant contact force), the ABA passes.
- `from_articulation.h` — `articulationToDiffModel` + `addColliderContacts` shape decomposition.
- `dual.h` — `softplus` / `sigmoid` and their `Dual` overloads.

## Tests added
- **`tst/physics/unit/diff_semiimplicit.cpp`** (new) — 4 probes (A–D below).
- **`tst/physics/integration/diff_contact_stability.cpp`** (+1) — `diff_contact_semiimplicit_humanoid_parity`.
- **`tst/physics/visual/diff_humanoid.cpp`** — `ENGINE_SEMI=1` / `ENGINE_SUBSTEPS=N` toggles to eyeball
  the semi-implicit path (and its stability at reduced substeps) against explicit.

---

## Findings

### 1. BUG — the compliant normal force is adhesive during separation (not clamped ≥ 0)
`groundContactSpatial` computes the net normal force as

```
Fn = k·softplus_β(pen)/β  −  c·vn·sigmoid(β·pen)
        └─ spring (≥0) ─┘     └─ dashpot, gated by penetration ─┘
```

The dashpot term is gated by `sigmoid(β·pen)` (penetration), **not** by the spring, and `Fn` is never
clamped to be non-negative. So when a penetrating contact point is **separating** (`vn > 0`), the
damping term can exceed the spring and `Fn` goes **negative** — the ground *pulls the body down*.
Physically impossible (a unilateral contact can only push).

**Measured** (`diff_ground_force_adhesion_on_separation`, defaults k=1e4, C=150, β=120, 2 mm penetration):

| contact-point vn | Fn        |
|------------------|-----------|
| 0 (rest)         | **+68.4 N** (repulsive — correct) |
| +2.0 m/s (separating) | **−99.6 N** (adhesive — wrong) |

Adhesion onset is at only **vn > 0.81 m/s** — easily reached during a bounce or a foot toe-off.

**Impact.** Benign at rest (vn≈0, damping/adhesion vanish — this is why the resting humanoid is fine).
But it (a) suppresses bounce by *actively pulling*, not just damping, and (b) — directly relevant to the
locomotion goal — **resists push-off / toe-off** and injects a spurious downward force + biased gradient
exactly when a walking/hopping policy tries to leave the ground.

**Fix (deferred), recommendation:** prefer a **Hunt–Crossley** damping model `Fn = k·sr − c·sr·vn`
(damping scaled by the penetration `sr`, so it vanishes at the surface and is **never adhesive**, and
stays smooth ⇒ differentiable). A hard `max(0, Fn)` clamp also removes adhesion but is non-smooth at 0
(hurts gradients) — use a soft clamp if going that route.

### 2. DESIGN SMELL — `SemiImplicit` damps the ENTIRE dynamics, not just the stiff contact
The predictor→corrector re-evaluates the **full** `diffForwardDynamics` (gravity torques, joint
coupling, *and* contact) at the predicted end-of-step state, then takes the real step with those forces.
That is backward-Euler-like for the whole articulated system, so it injects numerical damping into the
**smooth** dynamics too — even with no contact at all.

**Measured** (`diff_semiimplicit_damps_smooth_dynamics`, contact-free pendulum, 10 s):

| integrator     | E/E₀     |
|----------------|----------|
| Explicit (symplectic) | **1.007** (bounded, ~conserved) |
| SemiImplicit   | **0.630** (loses 37% of its energy) |

**Impact.** If semi-implicit is enabled, a policy's limb swings are artificially damped and gradients
are biased toward dissipation everywhere, not only near contact. The implicitness is *only* needed for
the stiff contact term. **Latent today** — explicit is the shipped default (Feature 3's multi-point
contact conditioned the humanoid enough that explicit is stable at k=1e4), so this only bites if someone
flips `ContactIntegration::SemiImplicit` on for extra stability headroom.

**Ideal fix (deferred):** IMEX — treat only the contact force implicitly, keep the smooth ABA dynamics
explicit/symplectic. Larger change; not needed unless a regime forces semi-implicit on.

### 3. VERIFIED GOOD (correctness/robustness confirmed by the new tests)
- **Free-fall equivalence** (`diff_semiimplicit_equals_explicit_in_freefall`): with a state-independent
  force (zero spin, no contact) SemiImplicit reduces **exactly** to Explicit — `|Δy| = 0`. Confirms the
  predictor→corrector is implemented consistently (no spurious extra force).
- **Determinism** (`diff_semiimplicit_deterministic`): the semi-implicit path (with friction) is
  **bit-identical** across runs — `|Δ| = 0`. Safe for reproducible/parallel training.
- **Humanoid rest parity** (`diff_contact_semiimplicit_humanoid_parity`): explicit and semi-implicit
  settle the passive ragdoll to the **same static equilibrium** (baseY 0.0889 m, penetration ≈ 0), with
  semi-implicit leaving *lower* residual KE (5.4e-7 vs 2.5e-6 J). The integrator choice does **not**
  distort the resting pose (expected — the contact spring balance is velocity-free at rest).
- **Smooth-contact primitives are overflow-safe**: `softplus`/`sigmoid` use the stable `log1p` forms and
  analytic `Dual` derivatives, so deep transient penetration (large β·pen) will not NaN the gradient.
- (Pre-existing, still green) gradient **through** semi-implicit contact matches finite differences; the
  semi-implicit stiffness-stability is ≥ explicit across the k sweep.

### 4. MINOR notes / smells
- **`contactRadius` + `contactPoints` both apply** if both are set (the ABA loop unions them). The
  converter only sets `contactPoints`, so no live double-count — but a hand-built model that sets both
  silently doubles the COM contact. Low risk; worth a comment or an assert.
- **ConvexHull colliders → crude 0.05 m COM-proxy sphere** in `addColliderContacts` (already a `TODO` in
  code). Irrelevant to `makeHumanoid` (capsules/boxes/spheres) but **relevant to the "adopt a richer
  mocap humanoid" goal** — a model with hull colliders would get poor/ungrounded contact.
- **Box → 8 radius-0 point contacts.** Fine for the humanoid's 2 feet; for a box-heavy mocap model this
  multiplies per-substep contact evaluations (a perf, not correctness, concern).

---

## Coverage gaps still open (suggested future tests, not done here)
- Gradient-through-**semi-implicit** on a multi-DOF / humanoid model (current semi-implicit gradient test
  is a single sphere).
- Quantify the adhesion's effect on a **bounce** (coefficient of restitution vs `groundC`) — ties Finding 1
  to an observable.
- End-to-end **deep-penetration transient** robustness (very high impact velocity) for both integrators.

## Bottom line for the training goal
The **shipped configuration (explicit, k=1e4, multi-point contact)** is in good shape: stable, deterministic,
correct equilibrium, differentiable, overflow-safe. Two issues to fix **before serious locomotion training**,
in priority order:
1. **Contact adhesion (Finding 1)** — will fight push-off and bias toe-off gradients. Fix with Hunt–Crossley.
2. **Semi-implicit whole-system damping (Finding 2)** — only if/when semi-implicit is turned on; otherwise latent.

Both fixes are deferred per the plan; this report + the new tests are the tripwires (the Finding-1 test
asserts the *current buggy* sign and should be flipped once `Fn` is made non-adhesive).

---

## RESOLUTION (2026-07-04) — findings 1, 2, 3 all fixed, tree green (142/0, ctest 7/7)

**Finding 1 — adhesion → non-adhesive contact.** The normal damping term is now gated off on
separation by an approach gate `σ(−groundDampBeta·vn)` (new `DiffModel::groundDampBeta`, default 40 s/m):
`Fn = k·sr − c·vn·σ(β·pen)·σ(−βv·vn)`. On separation (vn>0) the gate → 0 so `Fn = k·sr ≥ 0` (never
pulls); on approach (vn<0) it → 1 so compression damping is intact (low bounce preserved); out of
contact `sr, gate → 0` so no phantom force. Smooth ⇒ still differentiable. **Measured**: at 2 mm
penetration, Fn(vn=+2) went **−99.6 N → +68.4 N**, Fn(vn=−2)=+236 N (approach damping), Fn(rest)=+68 N.
Guard: `diff_ground_force_non_adhesive`. (Chosen over Hunt–Crossley, which — matched to the same
damping strength near the surface — is actually adhesive at low speed.)

**Finding 2 — whole-system damping → IMEX (contact-only implicitness).** `diffSubstep`'s SemiImplicit
branch is now IMEX: an explicit predictor gives the predicted state, then the corrector evaluates the
**smooth dynamics at the current state** and the **stiff contact force at the predicted state**
(`computeContactForcesWorld` + a new optional `extContactWorld` arg on `diffForwardDynamics` that
projects the predicted world-frame contact force into the current link frame). With no contact the
contact term is zero, so IMEX collapses exactly to the explicit symplectic step. **Measured**:
contact-free pendulum E/E₀ over 10 s **0.630 → 1.0066 (= explicit)**; stiff-contact stability retained
(semi ≥ explicit, both to k=8e4); gradient-through-semi-implicit still matches FD. Guard:
`diff_semiimplicit_imex_preserves_smooth_energy`.

**Whole-system damping decision** (answering "should we add another method?"): **not numerical — a
physical, optional one.** Added `DiffModel::jointDamping` (viscous `τ = −b·q̇`, default 0 = no behavior
change), applied as an explicit force in the ABA. Being a real force it is timestep-independent and
differentiable, and can be matched to the production backend so the diff and RL sims agree.
**Measured**: b=0 conserves (E/E₀=1.007), b=0.5 dissipates to 0.167, and **identical at h and h/2**
(0.1667) ⇒ timestep-independent (physical, not an integrator artifact). Guard:
`diff_joint_damping_physical`. The diff humanoid does not *need* it for stability (IMEX handles the
stiff contact; explicit smooth dynamics are energy-bounded) — it's there for realism / settling free
DOFs / RL-sim parity.

**Finding 3 — minor.** The contact set is now unambiguous: `linkGroundContactWorld` (and the FD contact
block) **prefer `contactPoints`, else fall back to the `contactRadius` COM shorthand** (never both ⇒ no
silent double-count). The ConvexHull crude-COM-proxy and box-8-point-count remain (still a `TODO`) —
intentionally deferred to the "adopt a richer mocap humanoid" task, where hull colliders actually appear.

Files: `include/engine/physics/diff/articulated.h` (contact model + IMEX + joint damping + contact-set
guard), `tst/physics/unit/diff_semiimplicit.cpp` (guards A/B/E flipped/added), `tst/physics/integration/
diff_contact_stability.cpp` (parity KE assertion relaxed). Deferred-fix items in `todo.md` marked done.

### Follow-up (2026-07-04) — contact stiffness/damping defaults promoted + IMEX stability characterization
After the adhesion fix exposed a real **bounce** (the old adhesion had been suppressing it) and a soft-k
**transient impact penetration** (~11 cm at k=1e4 — visibly clipping the floor), the `diff_humanoid`
visual was tuned and the values promoted to the engine `DiffModel` defaults: **`groundK` 1e4→4e4**
(transient penetration ~11 cm→~3 cm, stops clip-through) and **`groundC` 150→1000** (over-damped ⇒
drop barely bounces, no adhesion). Verified at the new stiffness: converter==reduced (9e-8 m),
gradient-through-contact==FD (**maxErr 1.4e-9**), rest-on-ground clean, rollout stable at substeps≥32
(DiffEnvironment uses 48). Note `jointDamping` (0.3 in the visual) stiffens the limbs so more impact
energy becomes COM bounce — hence the high `groundC`.

**IMEX stability characterization** (surfaced while re-verifying): because IMEX now treats *only* contact
implicitly (correct — no whole-system damping), `SemiImplicit` is **not strictly more stable than
explicit** at very coarse substeps; the old version's extra stability came from the (wrong) whole-system
backward-Euler damping. At moderate damping (C=150) both are stable to k=8e4 at substeps 32/48; at the
aggressive C=1000 the explicit predictor inside IMEX is fragile at substeps≤16. This is fine in practice —
**explicit is the default and is stable at the shipped config (k=4e4, C=1000, substeps≥48)** used by
training/visual. `diff_contact_semiimplicit_stability` was reframed accordingly (assert both cover the
shipped k=4e4 at training substeps, no strict dominance claim); parity test uses the shipped defaults.
