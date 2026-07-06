# Reduced-backend contact PGS — analysis & optimization plan

Dated note (2026-07-04). Follows the sparse-H contact optimization
([2026-07-04-reduced-coordinate-backend.md](2026-07-04-reduced-coordinate-backend.md)). After
sparse-H removed the dense `H⁻¹`, profiling showed that for a humanoid **lying flat** on the ground
the **PGS iteration**, not `H`, dominates the contact solve. This note records why and what to do.

## What the solver does now

`solveContacts` finishes with a **projected Gauss-Seidel / sequential-impulse** loop (20 iters). Per
contact it precomputes a normal + two friction-tangent Jacobian rows, each `H⁻¹Jᵀ` (sparse solve),
and a **scalar diagonal** effective mass `A_ii = Jᵢ H⁻¹ Jᵢᵀ`. Each iteration, per constraint:
```
vn  = dot(Jn, qd)                    // O(ndof)
Δλ  = clamp(λ − (vn + bias)/A_ii)    // λ≥0 normal; |λ|≤μλn friction (square clamp, per-tangent)
qd += JnHi · Δλ                      // O(ndof)
```
Only the *diagonal* of the contact-space system is built; off-diagonal coupling propagates
implicitly through the shared `qd`. Cost ≈ **O(iters · C · ndof)**, `C` = contacts × 3 directions.

## Why the many-contact case is PGS-bound

A humanoid lying flat has ~14 bodies all touching → `C ≈ 84`, `ndof = 27`, `iters = 20` ⇒ ~90k
mul-adds/substep, swamping the ABA (~few k) and the sparse factorization (~1k). Standing/walking
(the RL case) only has feet contacts (`C ≈ 8–16`) and is already cheap.

## Options and honest tradeoffs

1. **Delassus / contact-space PGS** — build `A = J H⁻¹ Jᵀ` (`C×C`) and iterate in contact space:
   `O(iters·C²)` instead of `O(iters·C·ndof)`. **Wins only when `C ≲ ndof`** (few contacts — the
   case sparse-H already made fast). For the flat humanoid (`C ≈ 84 > ndof = 27`) building the `C×C`
   matrix costs `~C²·ndof ≈ 190k` — *worse* than the current PGS. **SKIPPED** (wrong tool here).
2. **Warm-starting** — cache per-contact impulses across substeps/steps and seed the next solve.
   Resting-contact impulses barely change, so PGS converges in far fewer iters (≈4–8 vs 20). Biggest
   bang-for-buck; attacks the `iters` factor directly. **IMPLEMENT.**
3. **Contact-manifold reduction** — cap/dedup contacts per (body, plane) to a representative set
   (≤4). Shrinks `C` at the source; most valuable for box/convex faces (prevents pathological
   counts). Limited effect on the humanoid (its contacts are spread across many distinct bodies,
   each already minimal — a correctness-preserving cap can't remove legitimate distinct-body
   support). **IMPLEMENT.**
4. **Block friction solve** — solve each contact's two friction tangents as a coupled **2×2** block
   with a proper **circular** cone projection (radius `μλn`), instead of two sequential per-tangent
   **square** clamps. Better physics (isotropic friction) *and* faster convergence per iteration.
   **IMPLEMENT.**

## Decision

Skip #1. Implement #2 + #3 + #4. Expectation: warm-starting is the primary lever for the
many-contact regime; the block friction improves convergence + friction fidelity; manifold reduction
is hygiene that mainly helps box/convex faces. Keep all E1/E2/E3 tests green and re-benchmark
(flat-humanoid step cost + reduced-vs-maximal throughput) before/after.

## Results (2026-07-04) — DONE (#2 + #3 + #4)

Implemented in `featherstone_world.cpp`:
- **#2 Warm-starting**: `impulseCache_` (keyed by a stable `contactKey` = link|plane|feature) persists
  per-contact impulses `(λn,λt1,λt2)` across substeps/steps; the solve seeds each row and applies the
  cached impulse to `qd` up front. Stale keys drop out each substep. This let the iteration count
  drop **20 → 12** at equal quality.
- **#3 Manifold reduction**: cap each `(link,plane)` group to the **4 deepest** contacts (stable-sort
  by group then penetration). Hygiene for box/convex faces.
- **#4 Block friction**: precompute the `2×2` tangent effective-mass block (added `Atc = Jt1·Jt2Hi`)
  and solve both tangents coupled, projecting the impulse onto the **circular** cone (radius `μλn`)
  instead of two sequential square clamps — isotropic friction + faster convergence.

Correctness preserved: sphere rests (no penetration), box holds at μ=1 / slides at μ=0.05, humanoid
ragdoll settles + PD-hold, env determinism (single + VecEnv parallel==serial, bit-identical). Note:
at 8 iters the aggressive random-action VecEnv diverged, so **12 iters** is the stable setting.

Benchmarks (before = sparse-H only; after = + #2/#3/#4):
| metric | before | after | Δ |
|---|---|---|---|
| flat-humanoid contact-solve | 0.989 ms/step | **0.732 ms/step** | **−26%** |
| reduced env-steps/s, N=1 | 3492 | **4276** | +22% |
| reduced env-steps/s, N=1024 | ~26.5k | **~31.3k** | +18% |

The many-contact flat case (which sparse-H could not help — PGS-bound) is now **26% cheaper**, driven
mostly by the 20→12 iteration cut that warm-starting + block friction made possible. **End-to-end vs
the original dense-inverse baseline: N=1024 16.5k → 31.3k env-steps/s ≈ 1.9×.**
