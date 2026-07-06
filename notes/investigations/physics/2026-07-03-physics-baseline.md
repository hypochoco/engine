# 2026-07-03 — Physics Performance Baseline (pre-scaling)

Point-in-time snapshot of physics step throughput **before** the Phase-2 scaling work
(GJK/EPA + SAP/static-BVH broadphase). Captured via `tst/physics_bench`. Purpose: a
before/after yardstick on the **same machine** — absolute numbers are hardware-, power/thermal-,
and build-type-dependent and are **not** portable.

## Method
- Scenario: a grid of free-falling spheres, spaced > 2r so **nothing contacts** during the run.
  This isolates the current cost center — the brute-force **O(n²) broadphase** (every dynamic
  pair runs a `sphereVsSphere` narrowphase each step) + O(n) integration. Contact-solver cost
  is excluded on purpose (measured separately once it matters).
- Backend: `realtime` sequential-impulse, `substeps = 1`, `velocityIterations = 8`.
- Timing: best-of-3 reps of the mean single-step wall time over 200 steps (n ≤ 1024) / 60 steps
  (larger), after a 5-step warm-up. `std::chrono::steady_clock`.
- Build: **Release/optimized** (`-DCMAKE_BUILD_TYPE=Release`). Debug is far slower and not a
  valid baseline.

## Results (2026-07-03, this dev machine — Apple)

Raw `PhysicsWorld::step()` — free-fall, no contacts:

| bodies | µs/step | ms/step | steps/sec | body-steps/sec |
|-------:|--------:|--------:|----------:|---------------:|
|     64 |    6.87 |  0.0069 |   145,666 |      9.32 × 10⁶ |
|    256 |  111.06 |  0.1111 |     9,004 |      2.31 × 10⁶ |
|   1024 | 1513.93 |  1.5139 |       661 |      6.76 × 10⁵ |
|   4096 | 24175.2 | 24.1752 |        41 |      1.69 × 10⁵ |

ECS-bridge step (`Schedule`: `stepSystem` + `syncSystem`, poses → Transform):

| bodies | µs/step | ms/step |
|-------:|--------:|--------:|
|    256 |  112.19 |  0.1122 |
|   1024 | 1637.14 |  1.6371 |
|   4096 | 24283.0 | 24.2830 |

## Analysis
- **Quadratic, as expected.** Each 4× in body count → ~16× in step time (256→1024→4096 all
  ~13–16×). `body-steps/sec` falls ~55× from n=64 to n=4096 — the throughput collapse a
  broadphase fixes.
- **The ECS bridge is essentially free.** step+sync via the scheduler adds ~0.1–1% over the raw
  step (256: 112.2 vs 111.1 µs; 4096: 24283 vs 24175 µs). The `syncSystem` query + bulk
  pose→Transform copy is not a bottleneck; the cost is all in the O(n²) broadphase.
- **The 100k milestone is infeasible today.** Extrapolating the quadratic:
  `(100000/4096)² × 24.2 ms ≈ ~14 s per step`. Real-time (16.7 ms) at 100k needs the broadphase
  to go roughly sub-linear-in-pairs — i.e. exactly Phase 2 (SAP/BVH, sub-`O(n²)` pair finding),
  plus (later) parallel worlds and possibly SIMD/compute.

## Targets for Phase 2 (to compare against this note)
- Break the O(n²): SAP or a dynamic BVH so no-contact broadphase is ~O(n log n) / O(n + pairs).
- Re-run `tst/physics_bench` after Phase 2; expect the 1024/4096 rows to drop by 1–2 orders of
  magnitude and `body-steps/sec` to stay roughly flat as n grows (instead of falling ~linearly).
- Add a contact-heavy scenario (settled pile) to baseline solver cost once broadphase is fixed.

## Reproduce
```
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target physics_bench
./build-release/tst/physics_bench
```
