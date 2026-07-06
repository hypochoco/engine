# 2026-07-03 — Physics test-coverage findings

Point-in-time record of bugs surfaced while expanding physics test coverage (unit edge cases +
complex integration tests). Both are **reported, not yet fixed** — pending a fix decision.

## Test coverage added
26 new cases (all green), organized by module/category:

- `physics/unit/sphere_box.cpp` — sphereVsBox: face / edge / corner / center-inside / rotated.
- `physics/unit/manifolds.cpp` — boxVsPlane counts (flat=4, edge=2, buried=4-deepest);
  capsuleVsPlane counts (lying=2, standing=1).
- `physics/unit/segments.cpp` — capsule-capsule (parallel, crossing, coincident/degenerate),
  coincident spheres, capsule-sphere end-cap; NaN/`isfinite` safety on the degenerate branches.
- `physics/unit/so3.cpp` — exp/log round-trips (incl. small-angle and near-π), full 2π spin
  returns to identity, box + capsule inverse-inertia formulas, `worldInvInertia` rotation.
- `physics/integration/dynamics.cpp` — restitution, Coulomb friction (slide vs damp), head-on
  elastic velocity exchange, three-box stack, broadphase equivalence (SAP == grid), kinematic.

Backend behaviours confirmed while writing: restitution combines via `min(eA,eB)` and only
activates above a **1 m/s** approach speed; friction combines via `sqrt(muA·muB)`.

## Bug 1 — Kinematic bodies never move
`backends/realtime/sequential_impulse_world.cpp`

- `createBody` sets `invMass = dynamic ? 1/m : 0`, so **Kinematic** bodies get `invMass == 0`.
- Position/orientation integration runs inside `forEachDynamic`, which skips `invMass == 0`.
- ⇒ A `BodyType::Kinematic` body with a `linearVelocity` stays put; only its (correct) immunity
  to gravity holds.

Evidence: `physics.kinematic_motion` — a kinematic box with `linearVelocity=(1,0,0)` is at
`x=0.000` after 1 s (expected ~1.0). Gravity correctly leaves `y=5`.

Proposed fix: integrate kinematic positions by their prescribed linear/angular velocity every
step (advance pose, no gravity, no impulse response; they still act as `invMass=0` in the
solver so they push dynamics but aren't pushed back).

## Bug 2 — Continuous detection (CCD) suppresses restitution
`backends/realtime/sequential_impulse_world.cpp` (speculative-contact path)

With `continuousDetection = true` (the **WorldDef default**), a fast approaching body gets
speculative contacts generated while still separated. Those constraints bleed off approach
velocity across substeps *before* the surface is reached, so by contact time `vn` is small and
`restitutionBias = e·(−vn)` is negligible → almost no bounce.

Evidence: `physics.restitution` — drop from center-y 3.0 (impact ~7 m/s), plane+sphere `e=1`:
- CCD **off**: rebound peak `3.059` (near-perfect elastic return). `e=0` → `0.582` (no bounce). ✅
- CCD **on**:  rebound peak `0.589` (bounce essentially gone). ❌

The test asserts the correct CCD-off physics and prints the CCD-on suppression.

Proposed fix (options):
- Capture each contact's restitution target from the approach velocity measured *before* the
  speculative braking is applied (e.g. at first detection), and re-inject it at resolution; or
- Don't let a speculative (separated, `penetration < 0`) constraint remove more normal velocity
  than needed to close the gap — i.e. keep the speculative bias as an upper bound on the
  *position* closure without draining the restitution budget.

## Not bugs (verified correct)
Friction (slide vs damp), elastic velocity exchange, 3-box stack stability, SAP==grid
equivalence (bit-identical here), and all collision/inertia/SO(3) unit cases behaved as
expected.

## Resolution (2026-07-03, fixed)
Both fixed in `backends/realtime/sequential_impulse_world.cpp`; full suite green (47/47, debug +
release), CTest 4/4.

**Bug 1 — kinematic motion.** Added `forEachMoving` (iterates `alive && type != Static`, i.e.
Dynamic + Kinematic) and used it for the position/orientation integration step; gravity still
uses `forEachDynamic` (dynamic only) and impulses still skip `invMass==0`, so kinematics advance
by their scripted velocity, ignore gravity, and are never pushed back. Also widened the swept-AABB
CCD guard to `type != Static` so fast kinematics are broadphased along their path. Verified by
`physics.kinematic_motion` (x: 0→1) and `physics.kinematic_pushes_dynamic` (a rising platform
carries a resting box; platform 0.25→0.75, box 1.0→1.5).

**Bug 2 — restitution vs speculative contacts.** In the normal-impulse solve, the *separated*
(speculative) branch previously targeted `penetration/dt + restitutionBias`; since a speculative
contact is detected when `gap ≈ approachSpeed·dt`, the `-gap/dt` term nearly cancels the approach
velocity and brakes the body to a standstill before impact, cancelling the bounce. Now the
separated branch targets the rebound velocity directly when the pair is bouncing
(`restitutionBias > 0 ? restitutionBias : -gap/dt`); the overlapping branch is unchanged
(`baumgarte + restitutionBias`). Verified by `physics.restitution`: e=1 drop now rebounds to
3.058 with CCD **on** (was 0.589) and 3.059 with CCD off; e=0 stays at 0.582.

**Regression investigated & explained.** After Bug 2's fix the CCD test jumped from y=0.600 to
6.600. Root cause: that test's bullet never set `restitution`, so it used the default 0.2 — the
old bug had silently suppressed it. With restitution working, the bullet correctly bounced
(~0.2·120 m/s) instead of resting. Not an engine regression; the test now sets bullet+box
restitution to 0 to isolate no-tunnelling, and asserts a bounded stop (0.55 < y < 1.6) → 0.600.

**Known tradeoff (documented, not a bug).** A speculative contact resolves at detection, so a
*very fast* bouncy body can rebound up to ~`approachSpeed·dt` (one speculative margin) before the
geometric surface. Negligible at realistic speeds / with substeps; inherent to cheap
speculative CCD without a true time-of-impact solve.
