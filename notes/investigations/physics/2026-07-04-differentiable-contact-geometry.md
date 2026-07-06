#  Differentiable contact geometry + stability — implementation note (2026-07-04)

Scopes three follow-on features for the differentiable reduced-coordinate engine
(`include/engine/physics/diff/`), triggered by the `diff_humanoid` visual showing residual
ground penetration. Companion to
[2026-07-04-differentiable-reduced.md](2026-07-04-differentiable-reduced.md) (Phase F). **This is
the implementation plan; we build all three now.**

The visual investigation isolated two independent defects:
- **Shape mismatch** — a link's ground contact is a SINGLE compliant sphere at its COM
  (`DiffLink::contactRadius`), but links are capsules/boxes that extend past that sphere, so
  elongated/boxy parts clip the plane. → Features **2** (multi-point mechanism) + **3** (shape-aware
  points).
- **Soft-contact squish** — the compliant contact rests at penetration `pen = supported_force / k`;
  softened `k=2500` (lowered for explicit stability) gives cm-scale sinking (measured: 8 cm under the
  load-bearing torso). Raising `k` re-introduces the explicit blow-up. → Feature **4** (stable stiff
  contact).

Current contact surface (for reference):
- `DiffLink { … double contactRadius = 0.0; }` — one COM sphere if `>0`.
- `DiffModel { bool contactGround; double groundK, groundC, groundBeta, groundMu, frictionVref; }`.
- `groundContactSpatial(md, comWorld, linVelWorld, angVelWorld, Rw, radius) -> V6` — smooth compliant
  normal `k·softplus_β(pen)/β − c·vₙ·σ(β·pen)` + regularized Coulomb friction at the contact point;
  returned as a link-frame spatial force.
- Applied in `diffForwardDynamics` pass 1: `pA[i] += −groundContactSpatial(…)`.
- `diffSubstep`: explicit ABA → semi-implicit Euler on velocities → SO(3) config integration.

Cross-cutting invariants every feature must preserve: **Scalar-generic** (`double` and `Dual<N>` via
the same template), **quaternion-free**, **smooth/differentiable everywhere** (gradients vs FD to
≤1e-5), deterministic, and header-only. Each feature ends on a green tree (build + full suite + ctest).

---

## Feature 2 — multi-point contact mechanism

**Goal.** Let a link carry several compliant contact spheres at arbitrary link-local offsets, so one
rigid body can touch the plane at multiple places (capsule ends, box corners) instead of one COM point.

**Data model** (`articulated.h`):
```cpp
struct ContactSphere { V3<double> offset{0,0,0}; double radius = 0.0; };  // COM/link-frame center
struct DiffLink {
    …
    double contactRadius = 0.0;                 // KEPT: shorthand ⇒ one sphere at COM (back-compat)
    std::vector<ContactSphere> contactPoints;   // NEW: explicit multi-point set
};
```
The contact loop treats the effective set as: `contactRadius>0 ? {offset 0, contactRadius}` **plus**
all of `contactPoints`. Keeping `contactRadius` avoids churning the `diff_contact.cpp` unit tests
(they set a single COM sphere) and the current converter.

**Math — generalize `groundContactSpatial` to a local offset.** For a sphere at link-local `o`,
radius `r`, with link COM at `comWorld`, world rotation `Rw`, COM spatial velocity `v` (link frame):
- center `pc = comWorld + Rw·o`; penetration `pen = r − pc.y`.
- lever (contact point − COM, world) `ℓ = Rw·o − r·n`, `n = +y`.
- contact-point velocity `vp = linVel_com + ω_world × ℓ` (⇒ correct `vₙ = vp.y` and slip `= (vp.x, vp.z)`
  — a strict generalization; for `o=0` the vertical lever gives `vp.y = linVel.y`, matching today).
- force `F` (world) from the existing smooth normal+friction law using `pen`, `vₙ`, slip.
- spatial force (link frame) `[Rwᵀ(ℓ × F); Rwᵀ F]`.
New signature: `groundContactSpatial(md, comWorld, v /*V6 link-frame*/, Rw, offset, radius)`; pass-1
sums it over the effective sphere set into `pA[i]`.

**API/helpers.** `DiffLink::addContactSphere(V3 offset, double r)`. No `DiffModel`-level changes.

**Validation** (`tst/physics/unit/diff_contact.cpp` additions):
- A capsule-proxy body with 2 offset spheres rests flat with **both** ends on the plane (no tilt),
  vs the single-COM sphere which lets an end dip — regression for the visual bug.
- Offset-sphere gradient (seed initial height/velocity) vs FD ≤1e-5 (the `ℓ`-dependent `vp` path).
- Momentum: a symmetric 2-sphere body dropped flat has zero net horizontal drift.

**Risk/effort.** Low. Mechanical generalization; the only subtlety is the lever/`vp` change (already
validated for `o=0`). ~½ day.

---

## Feature 3 — shape-aware contact points (capsule / box vs plane)

**Goal.** Populate Feature-2 contact points from each collider so shapes rest *on* the surface. Builds
directly on Feature 2 (it just fills `contactPoints`). Done in the converter
(`from_articulation.h`) + optional standalone helpers.

**Per-shape decomposition** (link/COM frame; matches the collider conventions in
`featherstone_world.cpp` — capsule long axis = local **y**):
- **Sphere** `(r)`: one point `{(0,0,0), r}`.
- **Capsule** `(r, hh)`: end caps `{(0,+hh,0), r}`, `{(0,−hh,0), r}` (+ optional mid `{(0,0,0), r}`
  for long capsules). Two end spheres of radius `r` are the *exact* capsule-vs-plane contact set (the
  cylinder rests along the segment; both endpoints share the load — no clipping at any tilt).
- **Box** `(ex,ey,ez)`: the 8 corners `{(±ex,±ey,±ez), 0}` (radius-0 point spheres; `pen = −corner.y`).
  Bottom-face corners bearing load ⇒ a flat foot rests without tipping. (Optionally inset corners by a
  small radius `ρ` for a rounded box + smoother gradients.)
- **ConvexHull**: defer — approximate by its AABB box for now (note it).

`radius = 0` spheres are already handled by the smooth law (`pen = −center.y`); confirm `softplus`
/friction behave at `r=0` (they do — `r` only enters via `pen` and the lever `−r·n`).

**Converter change.** `articulationToDiffModel(def, groundContact=false)`: when enabled, fill
`contactPoints` per body from its collider via the decomposition. Replace the current
`footContactRadius` foot-only path with: `enum class DiffContact { None, Feet, All }` (or a bool +
"feet only" flag) so callers pick feet-only (controlled-walking RL) vs all-body (ragdoll/among falls).
Feet-only stays the default for the RL humanoid; all-body is what the visual/ragdoll wants.

**Validation** (`diff_validation.cpp` / `diff_contact.cpp`):
- Capsule laid horizontal rests with axis parallel to and at height `r` above the plane (both ends
  down), tilted capsule rests on the lower end at the right height — no clip-through.
- Box (foot) rests flat, all 4 bottom corners near `pen≈mg/(4k)`, no tipping under vertical load.
- Humanoid all-body contact: settled `minRenderedBottom > −(soft penetration)` — i.e. clipping is now
  ONLY the soft-`k` squish (Feature 4's job), not shape mismatch. Tighten `diff_humanoid_rests_on_ground`.
- All shape gradients vs FD ≤1e-5.

**Risk/effort.** Low–moderate. Box corners multiply contact-point count (8/body) ⇒ perf note below.
~1 day incl. tests + wiring the converter enum.

---

## Feature 4 — stable stiff (semi-implicit) contact

**Goal.** Support a stiff `k` (→ mm-scale penetration) that stays stable at the env timestep
(`h≈1 ms`, `substeps≈16–64`), removing the soft squish. Currently the explicit substep goes unstable
for `k ≳ (stability limit)` because the compliant force is evaluated at the start-of-step state.

**The physics of the blow-up.** A contact is a stiff spring–damper on the operational-space (effective)
mass `m⊥`; explicit (forward-Euler) integration of a spring is stable only for `h < 2/ω`,
`ω=√(k/m⊥)`. Stiff `k` ⇒ tiny `h`. Implicit/linearly-implicit integration of the spring-damper is
unconditionally (or far more) stable, at the cost of added numerical damping.

**Recommended approach — linearly-implicit (IMEX) contact velocity update.** Split the substep into
the smooth articulated bias (explicit, cheap via ABA) and the stiff contact (implicit):
1. ABA gives the contact-free generalized/base acceleration ⇒ predicted velocity `qd*`.
2. For the contact force `Q_c(q, qd) = Σ_k J_kᵀ f_k` (smooth normal+friction), form the local
   linearization in velocity `∂Q_c/∂qd` (the damping + friction slope) and position `∂Q_c/∂q`
   (the spring slope, via `pen`), then solve one implicit velocity step
   `(A/h − ∂Q_c/∂qd − h·∂Q_c/∂q) Δqd = Q_c(q,qd*)` where `A` is the articulated inertia. Update
   `qd_{t+1} = qd* + Δqd`.
   - The needed `∂Q_c` blocks are analytic (the contact law is closed-form) and local to contacting
     links; the solve is small. **Differentiable**: it's a linear solve (forward-mode `Dual` flows
     through `invertSmall`/`solveM6`, as elsewhere).
   - `A` (mass matrix) can be obtained via CRBA (mirror `featherstone_world.cpp`) or reuse the ABA
     articulated inertias; scope the exact assembly during implementation.
3. Semi-implicit spring uses the predicted post-velocity position (`pen` at `q_t` with the implicit
   `Δqd` correction), which is what buys the stability.

**Pragmatic fallback (ship first, lower risk).** If the full IMEX solve proves too invasive to keep
clean/differentiable in one pass:
- (4a) **Implicit normal/friction damping only**: use `vₙ_{t+1}`, `slip_{t+1}` (a per-contact scalar
  implicit update on the effective mass) — cheap, no global solve, lets `c` and `k` rise moderately.
- (4b) **2-pass semi-implicit substep**: explicit velocity predictor → recompute contact at the
  predicted position → corrector. ~2× substep cost, no linear algebra, moderate stability gain.
Target with the fallback: ≥4–10× stiffer stable `k` (penetration from ~8 cm → ~1–2 cm). Target with
full IMEX: `k≈3e4–1e5` (penetration ~1–3 mm) stable at `substeps≤32`.

**API.** A `DiffModel` switch `enum class ContactIntegration { Explicit, SemiImplicit }` (default
`SemiImplicit` once validated) so we can A/B and keep the explicit path for reference/tests. Raise the
default `groundK` only after the implicit path is stable at the higher value.

**Validation** (new `tst/physics/unit/diff_contact_stability.cpp` + extend `diff_validation.cpp`):
- Stiffness sweep: implicit path stays finite + settles for `k` up to the target at `substeps≤32`
  (compare vs explicit's failure point). Report penetration vs `k`.
- Penetration: humanoid all-body contact settles with rendered bottoms within a few mm of the plane.
- **Gradients through the implicit contact vs FD ≤1e-5** (the critical check — the implicit solve must
  stay differentiable), on the sphere-rest, sliding-friction, and hopper cases.
- Energy: no spurious injection at contact onset across the stiffness range.
- Determinism preserved.

**Risk/effort.** High — this is a solver-design piece. Main risks: (i) assembling `A`/`∂Q_c` while
keeping the ABA structure + AD clean; (ii) friction coupling in the implicit solve (may treat friction
explicitly, normal implicitly — a common, stable split); (iii) added numerical damping altering
physics (validate energy/rolling). Mitigation: land the fallback (4a/4b) first for an immediate win +
green tree, then attempt full IMEX behind the `ContactIntegration` switch. ~2–4 days.

---

## Ordering, dependencies, milestones

```
2 (multi-point mechanism)  ──▶  3 (shape-aware points)  ──▶  tighten rest tests
                                                    │
4 (semi-implicit stability) ────────────────────────┘  (orthogonal to 2/3; needs 3 to judge penetration)
```
- **M1 — Feature 2**: `ContactSphere` + generalized `groundContactSpatial` + offset-sphere tests. Green.
      **DONE (2026-07-04):** `ContactSphere{offset,radius}` + `DiffLink::contactPoints`/`addContactSphere`
      (kept `contactRadius` COM shorthand); `groundContactSpatial` now takes a link-local offset (contact
      point `Rw·o − r·n`, contact-point velocity `v_com + ω×ℓ`) — exactly back-compatible at `o=0` (all
      prior contact tests unchanged). Pass-1 sums over the effective sphere set. Tests
      (`diff_contact.cpp`): offset-sphere gradient vs FD 8 digits; two spheres share load (rest at
      `r−mg/2k`), self-right a 0.10 rad tilt→0, no drift. Perm. benchmark `diff_contact_cost` in
      `diff.cpp` (~36 ns/point). Suite 127/0.
- **M2 — Feature 3**: shape decomposition in the converter + `DiffContact::{None,Feet,All}` +
  capsule/box rest + gradient tests; update the `diff_humanoid` visual to use `DiffContact::All`
  (drop its local hand-rolled radii). Green.
      **DONE (2026-07-04):** `enum DiffContact{None,Feet,All}` + `addColliderContacts` (sphere→COM,
      capsule→2 end-caps @ ±hh radius r, box→8 corners radius 0); `articulationToDiffModel(def,
      DiffContact)` replaces the `footContactRadius` double; `DiffEnvironment` ctor + all call sites
      updated. Visual + `diff_humanoid_rests_on_ground` now use `DiffContact::All` (no hand-rolled
      radii). Tests: box rests flat at `ey−mg/4k` tilt 0; capsule (horizontal) rests at `r−mg/2k`;
      converter decomposition counts (box 8 / capsule 2 / sphere 1; Feet=2 bodies, All=14); feet-corner
      contact gradient still == FD (6e-10); ragdoll `minBodyY` −1.6→**+0.041** (rests on plane). Cost
      (`diff_contact_cost`): none 0.00287 / feet 16pt 0.00319 (+11%) / all 64pt 0.00444 (+55%), ~26
      ns/pt — matches projection. Suite 130/0.
- **M3 — Feature 4a/4b (fallback)**: semi-implicit substep behind `ContactIntegration` switch; stiffen
  `k`; stability + gradient + penetration tests. Green.
      **DONE (2026-07-04):** implemented the 2-pass predictor→corrector (`ContactIntegration::{Explicit,
      SemiImplicit}`, `DiffModel` field; `diffSubstep` refactored into `diffApplyAccel`/
      `diffIntegrateConfig` + the semi-implicit branch that re-evaluates dynamics at the predicted
      state). Validated in `diff_contact_stability.cpp`: semi-implicit is **differentiable** (gradient
      through it == FD to 8 digits) and **at least as stable as explicit** across a `k` sweep.
      **KEY FINDING:** Feature 3's multi-point contact already conditioned the humanoid so well that
      the EXPLICIT integrator is stable to `k=8e4` at substeps 16–48 (the earlier blow-up was the
      feet-only single-COM-sphere geometry funneling whole-body load through 2 points). ⇒ restored the
      briefly-softened `groundK` 2.5e3→**1e4** (ragdoll rest `minBodyY` 0.041→0.053, contact
      penetration ~0), kept **Explicit as the default** (semi-implicit gives equal stability here and
      occasionally more penetration). Semi-implicit stays available behind the switch for harder
      regimes (heavier bodies, larger h). Suite 132/0.
- **M4 — Feature 4 full IMEX**: **NOT NEEDED for the humanoid** — Feature 3 + restored `k` gives ~mm
  penetration with the explicit path. Reserve the full linearly-implicit solve for genuinely stiffer
  cases if they arise; the `ContactIntegration` switch + validated semi-implicit path are the hooks.

### Contact bounce tuning (2026-07-04)
Bounce = under-damped compliant normal response, set by `groundC` relative to critical damping
`2√(k·m_eff)` (≈200 for a ~1 kg / k=1e4 contact). Measured sphere drop (k=1e4): rebound
c=40→0.152 m, c=80→0.070, c=160→0.029, c=320→0.019 (maxPen 0.018→0.001); c≳640 overdamped.
Higher `groundC` cuts BOTH bounce and transient penetration. ⇒ default `groundC` **80→150**
(near-critical, >50% less bounce, still stable explicit at substeps 16). Note: the low activation
sharpness `β=120` makes the compliant equilibrium sit slightly ABOVE the geometric surface (contact
felt ~1 cm early) — raise `β` for a crisper surface (at some smoothness/stability cost); raise `k`
for less steady penetration; raise `groundC` for less bounce. `ContactIntegration::SemiImplicit`
allows pushing `groundC` higher without the explicit damping-stability limit.

## Perf notes
- Contact cost scales with total contact points × substeps. All-body box decomposition = up to
  8 pts/body × 14 = 112 pts (vs 2 today). Benchmark contact-on step cost after M2 (`diff.cpp`); if it
  bites batch throughput, cull to bottom-face corners / cap points per body.
- Feature 4's per-substep solve adds cost but should permit fewer substeps (stiffer, stable) — net
  effect measured in `diff.cpp` (substeps × solve vs today's substeps).
- Forward-mode `Dual` cost grows with any new linear solves; keep contact-space systems small/local.

### Measured baseline (2026-07-04, single-thread RelWithDebInfo, humanoid diff substep)
Scope reminder: these features touch ONLY the differentiable engine. The production reduced/realtime
backends `VecEnv` uses for PPO-style rollouts have a separate contact solver and are **unaffected**
(≈31.8k humanoid env-steps/s unchanged). Only the SHAC/analytic-gradient path (`diffSubstep`,
`rolloutGradient`, `DiffEnvironment`) is affected.

| config | ms/substep | Δ |
|---|---|---|
| no contact | 0.00296 | — |
| feet (2 COM spheres, today) | 0.00312 | +5% |
| all 14 bodies (1 COM sphere each) | 0.00349 | +18% |

⇒ **~34 ns per contact point** vs a ~2.9 µs contact-free ABA substep (each point ≈1%). Projections:
- **F2 (mechanism):** ~free; cost is linear in point count.
- **F3 feet-only (RL default):** feet→boxes = 16 corner points (or 8 bottom-only) ⇒ **~+8–15%** of the
  substep. **F3 all-body (ragdoll/visual only):** ~60–80 points ⇒ **~2×** the contact-free substep.
- **F4:** per-substep implicit solve costs more per substep (2-pass ≈2× ABA; full IMEX = ABA + a small
  contact-space solve), BUT lets contact envs drop from ~48 → ~16 substeps ⇒ **net ~1.5× faster per
  control step** with mm (not cm) penetration. Net win.
- **Gradient path:** forward-mode `Dual<N>` scales all op cost by ~N, so extra points + F4 solves scale
  the gradient the same way (today NA=21 ≈ 20 ms / 8-step rollout); computed far less often than
  forward steps, and full-action gradients want reverse-mode/BPTT regardless.
- Knobs if it bites: bottom-corners-only per box, cap points/body, feet-only for training. A permanent
  contact-cost + contact-on-step benchmark lives in `tst/physics/benchmark/diff.cpp` to gate this.

## Validation matrix (must stay green)
| Property | Test |
|---|---|
| offset-sphere gradient vs FD | `diff_contact.cpp` (F2) |
| capsule/box rest on plane, no clip | `diff_contact.cpp` / `diff_validation.cpp` (F3) |
| shape gradients vs FD ≤1e-5 | (F3) |
| humanoid rests, bottoms ~mm from plane | `diff_humanoid_rests_on_ground` (F3→F4) |
| implicit stable at stiff k, substeps≤32 | `diff_contact_stability.cpp` (F4) |
| gradients through implicit contact vs FD | `diff_contact_stability.cpp` (F4) |
| converter still == reduced backend (no-contact) | `diff_validation.cpp` (unchanged) |
| determinism, energy sanity | F1/F2 suite |
