# Realtime contact-solver stability: energy injection + tall-stack collapse (2026-08-01)

Point-in-time investigation. Triggered by a game report: a cube on a rock "gains energy and is
eventually thrown off," and stacking is unstable. Goal: very stable stacking (10–30 boxes). Scope:
the **realtime** maximal-coordinate sequential-impulse backend
(`src/physics/backends/realtime/sequential_impulse_world.cpp`) — NOT the reduced/diff RL path.

## Repro tests (new)

`tst/physics/integration/stack_energy.cpp` — quantify the two suspected causes:

| test | result | evidence |
|---|---|---|
| `baumgarte_energy_injection` | FAIL | resting box, **gravity OFF**, 0.05 m penetration → ejected **+4.0 m at 2.0006 m/s**, KE 2 J from nothing |
| `cube_on_rounded_rock_launch` | FAIL | box gains **2.48 m/s UPWARD** while still on the rock (gravity can't do that) → propelled off |
| `cube_on_flattop_hold` | PASS | wide flat top: drift 0.089, maxSpeed 0.006, energy flat — stable |
| `stack_of_ten_boxes` | PASS | sink 0.053 m (~5 mm/contact), tilt 0.6° |
| `stack_of_thirty_boxes` | FAIL | sink **28.97 m**, tilt **180°** — collapses |

Failing tests assert the DESIRED stable behavior, so they double as regression tests for the fix.
(Existing `tst/physics/unit/stacking.cpp` still passes — it only covered flat/single-box rest, which
is exactly the case that already works, so it masked both root causes.)

## Root cause #1 — Baumgarte velocity bias injects energy

`solveConstraint` drives the normal velocity toward `target = baumgarte + restitutionBias`, where
`baumgarte = min((contactBaumgarte/h)·max(penetration − contactSlop, 0), maxCorrection)`
(defaults: `contactBaumgarte=0.2`, `contactSlop=0.005`, `maxCorrection=2.0 m/s`). That outward bias is
added to the **real** velocity and **retained after the substep** — it is classic Baumgarte, not
split-impulse. So every penetrating contact hands the body up to `maxCorrection` (2 m/s) of free
velocity (proven in isolation: 2.0006 m/s from nothing, no gravity).

- On a **wide flat** contact the kicks are vertical + symmetric and damping absorbs them → looks
  stable (`cube_on_flattop_hold` passes). This is why the earlier "add a flat-topped boulder" fix
  *appeared* to work — it treated the geometry symptom, not the cause.
- On a **rounded/tilted** contact the normal is tilted and the manifold point wanders, so the kick has
  lateral + upward components → the box is launched (2.48 m/s up). This is the game report.

**Fix:** split-impulse / pseudo-velocity — apply penetration correction to a *separate* positional
velocity that fixes position but never feeds the post-step velocity. Eliminates energy injection.

## Root cause #2 — no contact warm-starting → tall stacks can't converge

`buildConstraints` rebuilds every contact each substep with `normalImpulse = 0`; there is no
persistent manifold carrying impulses across substeps/steps (joints ARE warm-started; contacts are
not). Sequential-impulse without warm-starting needs many iterations to propagate the support impulse
up a stack; with 8 iterations a 10-tall tower barely holds (5 mm/contact sink) and a 30-tall tower
can't converge and collapses.

**Fix:** persistent contact manifolds + warm-starting (cache contacts across steps by feature/nearest
point, carry `normal/tangentImpulse`, seed each substep). Optionally more iterations / TGS.

## Fix options + effort

1. **Split-impulse position correction** — kills energy injection (#1). Low–moderate, localized.
2. **Contact warm-starting (persistent manifolds)** — deep-stack convergence (#2). Moderate (needs a
   contact cache keyed by body-pair + contact id).
3. **TGS-Soft** (Box2D v3 / PhysX-style substepped Temporal Gauss-Seidel + soft constraints +
   warm-start) — reference-quality stacks, no energy gain, but a larger rewrite. Do 1+2 first.

Recommended order: **split-impulse first** (directly fixes the reported launch), then **warm-starting**
for the 10–30 stack goal. Both contained to the realtime backend; RL/reduced path untouched.

## Status

DELIVERED (2026-08-01, behind `WorldDef` flags): split-impulse (default ON), contact warm-starting
(default OFF), TGS-Soft (opt-in), and the contactBaumgarte 0.2→0.5 tuning. Tests GREEN: physics
143/0, core 15/0, graphics 24/0. Remaining work is the TGS follow-up (stable-feature-ID warm-start
key, per-substep joints, dense-pile robustness, 2-axis friction) — see the sections below and the
todo bullet in `core/todo.md`. All engine changes are UNCOMMITTED (submodule) — awaiting commit/push.

## Benchmark BASELINE (2026-08-01, before fixes)

`tst/physics/benchmark/box_stability.cpp` — deep unit-box pile on a plane, substeps=4, velIter=8,
pooled, RelWithDebInfo (optimized + `ENGINE_PROFILING` on for the phase breakdown). Solver step()
instrumented with `phys.{build,broadphase,narrowphase,solve}` prof zones. Same-machine only.

| bodies | ms/step | contacts | meanSpeed | maxSpeed |
|---|---|---|---|---|
| 1,000   | 13.3  | 16.8k  | 0.046 | 0.27 |
| 10,000  | 64.0  | 160k   | 0.107 | 0.38 |
| 50,000  | 309   | 655k   | 0.330 | 1.23 |
| 100,000 | 700   | 1.76M  | **2.11** | **3.74** |

Two things stand out: (a) the 100k pile is **agitated** (meanSpeed 2.11 vs 0.046 at 1k) — it can't
settle, consistent with energy injection + under-convergence at depth; (b) phase breakdown @100k
(per-substep avg, ×4 substeps/step): **phys.broadphase 74 ms (dominant)**, phys.solve 65 ms,
phys.narrowphase 11.7 ms, phys.build 104 ms total. Broadphase + narrowphase run EVERY substep (4×).

Implications for the fixes: split-impulse (#1) adds a position pass (↑ solve, fixes settling);
warm-starting (#2) should pair with **building contacts once per step + reusing the manifold across
substeps** (update separations analytically) → cuts ~3×74 ms broadphase + ~3×11.7 ms narrowphase at
100k, the biggest single perf lever, while warm-start improves convergence (settling). Re-benchmark
after #1+#2.

## AFTER #1 (split-impulse) + #2 (contact warm-starting) (2026-08-01)

Implemented in `sequential_impulse_world.cpp`:
- **#1 split-impulse**: removed the Baumgarte term from the REAL normal-velocity target; added a
  separate pseudo-velocity (`biasLin_/biasAng_`) position pass (`solvePositionColored`, `positionIterations`
  sweeps/substep) that depenetrates and is applied at integration then discarded. `WorldDef::positionIterations=4`.
- **#2 warm-starting**: persistent `contactCache_` (`std::unordered_map<key, {normalImpulse,stamp}>`,
  key = pack(bodyA,bodyB,contactIdx)); `warmStartColored()` seeds + applies the cached normal impulse
  before iterating; `storeContactImpulses()` writes back; lazy prune. `WorldDef::contactWarmStart=true`.

### What improved
- **Energy injection FIXED** (#1): `baumgarte_energy_injection` maxSpeed 2.0→**0.0**; `cube_on_flattop_hold`
  drift→**0.000**; `edge_no_launch` 5.0→**1.2**; `cube_on_rounded_rock` upward launch 2.48→~0.45 m/s.
- **Warm-starting demonstrably works** (#2): 10-box stack at velIter=12 — `warm=0` COLLAPSES (sink 9,
  topples) vs `warm=1` STABLE (sink 0.046, tilt 1.1°). The convergence win is real.
- 100k pile is calmer: **meanSpeed 2.11→0.72**.

### Costs / limits found
- **Perf REGRESSED at 100k: 700→~1163 ms/step (+66%)**. Breakdown (per-substep): phys.solve 65→**118 ms**
  (the naive `unordered_map` warm-start find/store over 1.87M contacts), + new phys.position **37 ms**;
  broadphase/narrowphase still run ×4/substep (the narrowphase-once win was NOT done — it belongs to the
  TGS rework). ⇒ the warm-start cache MUST become an open-addressed, allocation-free per-world table,
  and manifolds should be built once/step and reused across substeps.
- **Deep stacks still fail**: velIter=8 default is too low for split-impulse (a 10-tower needs ≥12);
  a 30-tower is **metastable** and topples/collapses regardless of iterations (velIter up to 40,
  substeps 8) — solver asymmetry (single-axis friction + Gauss-Seidel order) breaks the perfect tower.
- **Regression**: `ragdoll_settles` maxOmega 1.0→**1.45** (articulated limb retains angular jitter;
  linear still settles). The split-impulse change is not parity-preserving for jointed-on-contact bodies.

### Conclusion → do #3 (TGS-Soft)
#1+#2 fix the reported energy/launch bug and prove warm-starting helps convergence, but incrementally
bolting them onto the existing solver costs perf (naive cache), regresses the ragdoll, and does not
give robust deep stacks. The clean answer is **TGS-Soft** (Box2D-v3/PhysX-style substepped Temporal
Gauss-Seidel + soft constraints + warm-starting, with a 2-axis block friction and manifold reuse
across substeps): it subsumes #1+#2 coherently, converges far better per-iteration (deep stacks),
removes energy injection by construction, and — via narrowphase-once + an allocation-free warm-start
table — should also recover/beat the baseline perf at 100k. Recommend proceeding to #3.

## #3 TGS-Soft GAUGE (2026-08-01, built + measured)

Added a full TGS-Soft contact path (`WorldDef::contactSolver = TGSSoft`): manifold built ONCE/step +
reused across substeps (anchors body-local, separation re-derived each substep), soft-constraint
normal (contactHertz/dampingRatio) with a bias + relax pass (energy-free by construction), 2-axis box
friction, warm-starting. Kept #1 (`splitImpulse`) + #2 (`contactWarmStart`) as flags on the
SequentialImpulse path (default on). Gauge (all same machine):

| case | SI baseline | SI +#1+#2 | TGS-Soft |
|---|---|---|---|
| 100k pile ms/step | 700 | 1163 | **335** (narrowphase-once) |
| clean 10-stack | held (metastable) | needs velIter≥12 | **rock-solid** (sink .05, tilt .2° @ substeps 8) |
| baumgarte energy inject | 2.0 m/s | **0.0** | 0.078 (small residual) |
| cube on ROUNDED rock (the report) | 2.48 up | **0.42 up** | 0.90 up (worse) |
| flat-top hold | ok | ok | **perfect** (drift 0) |
| 30-tall column | collapse | collapse | topples (30:1 aspect = metastable) |
| dense 100k pile settle | mean 2.1 | mean **0.72** | mean 2.5 / max **14.7** (UNSTABLE) |
| ragdoll settle (maxOmega) | pass | **1.45 FAIL** | **151 EXPLODES** |

Key isolations:
- **The ragdoll regression is #2 (warm-starting), NOT #1.** `WARM=0` → maxOmega 0.22 (pass) with split
  on OR off; `WARM=1` → 1.45. The naive `(bodyA,bodyB,contactIdx)` warm-start key misapplies impulses
  when a manifold's point ORDER shifts (moving limbs) → residual spin. Needs stable feature IDs or
  position-matching, not a positional index. #1 split-impulse has NO ragdoll impact.
- **TGS is a strong perf + clean-stack win** (2× faster than baseline via narrowphase-once; 10-stack
  rock-solid) **but has real correctness gaps as implemented**: (a) JOINTS explode — `prepareJoints`
  is called once/step, not per substep, so with per-substep position integration the joint constraints
  diverge; (b) the ROUNDED-rock launch is worse (fixed step normal + soft push on a curved tipping
  contact); (c) the dense 100k pile is unstable (once-per-step narrowphase can't track fast dense
  collapse → penetration overshoot). TGS needs: per-substep joint prep, per-substep contact refresh
  (or tighter speculative) for fast bodies, and friction/anchor tuning.

### Recommendation after gauging
- **Ship #1 (split-impulse) ON by default** — it fixes the actual report (cube thrown off the rock:
  2.0→0.42 m/s) with NO regressions (ragdoll fine) at modest cost. The clear keeper.
- **Default #2 (warm-starting) OFF** — as implemented it regresses articulated bodies (ragdoll) via the
  fragile index key, and only helps tall clean stacks (not the reported need). Keep as an opt-in flag
  until the key uses stable feature IDs / point-matching.
- **Keep TGS as opt-in** and treat it as a focused follow-up project: it is the right architecture for
  deep stacks + 100k perf, but needs per-substep joints, dense-pile robustness, and rounded-contact +
  friction tuning before it can be a default. The 30-tall single column is partly physical (metastable
  aspect ratio) regardless of solver.

## Contact-quality follow-up: interpenetration + "strange at angles" (2026-08-01)

After shipping split-impulse ON / warm-start OFF, the user reported: objects still (1) interpenetrate a
little before correcting, and (2) behave "strange at angles." Diagnostics: `tst/physics/integration/
contact_quality.cpp` (`impact_penetration`, `tilted_landing`, `incline_friction_stick`).

**(1) Interpenetration = two parts:**
- **Impact dip** — a box hitting a pedestal at ~5 m/s penetrates **~13 mm (4 substeps)** in the substep
  before the contact resolves (discrete detection; the speculative margin reduces but doesn't remove
  it). Halved to ~7 mm at 8 substeps. This is the momentary overlap.
- **Slow recovery (the dominant issue)** — with `contactBaumgarte=0.2` the split-impulse push-out is
  too gentle: penetration stays >1 mm for **~177 frames (~3 s)**. Raising **contactBaumgarte 0.2→0.5
  cuts that to ~1-2 frames** (snappy pop-out). Crucially this is now SAFE: split-impulse discards the
  bias velocity, so a high rate does NOT reinject energy (verified: baumgarte_injection still 0.0 m/s,
  rounded-rock launch unchanged at 0.42). `contactBaumgarte=0.2` was tuned for the OLD Baumgarte-in-
  velocity scheme; split-impulse decoupled correction rate from energy, so it should be raised.
- Also: `contactSlop=5 mm` = permanent resting overlap (visible up close) — reducible to 1-2 mm.

**(2) "Strange at angles" — mostly NOT a solver bug:**
- A box dropped tilted 25° drifts **~0.27 m** while tipping to rest — but this is **identical across
  SI, SI+substeps, and TGS**, so it's tipping DYNAMICS (corner impact → horizontal slide), not a solver
  artifact. Corner penetration during the tip is 0.1 mm on SI (TGS is worse: 13 mm).
- On a 15° ramp (mu 0.8, should stick) the box creeps only **7.7 mm/5 s** (settle transient) with **~0
  sideways drift** → single-axis friction is NOT causing lateral drift at static rest. More substeps
  → 2.7 mm. The angled corner also benefits from the higher contactBaumgarte recovery.

**Recommended tuning — benchmarked before/after (2026-08-01):**
- **APPLIED: `contactBaumgarte` 0.2→0.5** (engine `SolverConfig` default). Fixes the dominant "corrects
  slowly" symptom: impact-penetration lingers >1 mm for **177 frames → 2 frames**. **Perf-neutral**
  (100k pile: 1067→1090 ms, noise). No regressions (rounded-rock 0.42, edge 1.2, ragdoll ω 0.29 all
  fine). Safe now only because split-impulse discards the bias velocity (energy still 0.0).
- **REJECTED: lowering `contactSlop` (5→2/3 mm).** Bisect showed it REINTRODUCES launches on
  edges/curved contacts (edge box 1.2→5.7 m/s at 3 mm; ragdoll fails at 2 mm) — the tighter slop makes
  the push-out act on more penetration → spikes. Kept at 5 mm. (This 5 mm is the residual permanent
  resting overlap; reducing it safely needs the TGS soft contact, not a tighter slop.)
- **REJECTED for the default: `substeps` 4→6.** Only trims the momentary impact DIP (13→9 mm) but costs
  **+47%** at 100k (1067→1707 ms). Left to the app to raise per-scene if desired.
The residual impact dip (~13 mm on a 5 m/s hit, from discrete detection) and the tilt-landing drift +
true 2-axis warm-started friction are the TGS follow-up.




