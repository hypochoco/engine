# CUDA port — measured CPU baseline + resolving blockers 2 & 3 (fixed-size ABA, flat model)

Dated (2026-07-08). Implementation follow-up to the [CUDA port review](2026-07-06-cuda-port-review.md).
Goal restated: make the engine **CUDA-capable alongside** the CPU backends — CPU+Metal on the Mac,
CPU+CUDA on a Linux/NVIDIA box, from one codebase. This session (a) measured the CPU baseline on the
dev machine and (b) resolved the two "straightforward" blockers the review flagged — the per-call heap
allocations in `diffForwardDynamics` (blocker 2) and dynamic topology (blocker 3) — as **CPU-only,
device-preparatory refactors** that keep every diff test green. No `.cu` yet (CUDA can't build on the
Mac); this de-risks the mechanical changes on the platform we can debug on.

## 1. Measured CPU baseline (Apple M3 Pro, 6 perf / 12 logical cores, build-release)

Note: this is the **dev machine**, not the training target. The review's A10G estimates stand; this is
the "before" for the refactors and the Amdahl-relevant single-thread number.

| Benchmark (`tst/benchmarks`) | Result |
|---|---|
| diff forward substep, `double`, humanoid 14-link/21-DOF | **0.0026 ms → ~385k substeps/s** (single-thread) |
| reduced backend substep (`float`), same rig | 0.0041 ms (the diff ABA is *leaner* than the maximal backend) |
| diff gradient, 8 control steps: forward | 0.323 ms |
| diff gradient, forward-mode `Dual<N>`, N = 1 / 4 / 21 seeds | 1.15 / 2.74 / 12.45 ms (~38× fwd at 21 seeds, ~linear in seeds) |
| maximal `step`, free-fall, 1024 bodies (uniform-grid) | 0.199 ms/step |
| parallel worlds, 64 × 1024 bodies, 12 workers | 7.31× over serial (3.9e7 body-steps/s) |

The **0.0026 ms/substep** figure is the "before" for blocker 2.

## 2. Blocker 2 — fixed-size-ify `diffForwardDynamics` (heap-free hot path)

**Problem (review §3.2):** the ABA allocated ~a dozen `std::vector`s **per call** (`Xup, IA, v, c, pA,
a, Rw, pos, Scol, U, Dinv, uv`), plus a `std::vector` in the returned `Accel`. Device hot code can't
touch the heap; this working set must live in stack/registers/local memory.

**Change (`include/engine/physics/diff/articulated.h`):**
- Added compile-time caps `kMaxLinks = 16`, `kMaxDof = 32` — snug over the built-in rigs (humanoid
  14/21, AMP 15/28). A model exceeding them trips an `assert`; bump the caps for bigger rigs.
- The ~dozen `std::vector<T> x(n)` locals → fixed-size stack arrays `T x[kMaxLinks]`.
- `Accel<S>::qddot`: `std::vector<S>` → `S qddot[kMaxDof]`.
- `Accel`, `diffForwardDynamics`, `computeContactForcesWorld` are **all internal to `articulated.h`**;
  the only external entry point is `diffSubstep(md, state, std::vector tau, grav, h)` (~40 call sites),
  whose signature is unchanged — **zero caller ripple**. `diffApplyAccel` reads `acc.qddot[k]` by index,
  unchanged.

**Sizing subtlety worth recording:** the math types (`V3/M3/V6/M6`) have in-class member initializers,
so `T x[kMaxLinks]` zero-initializes **all** `kMaxLinks` elements regardless of `{}`. With a loose cap
(32) the `Dual<21>` gradient path regressed ~5% (initializing 32 links when 15 are used). Tightening to
16 restored gradient parity **and** lowers eventual GPU per-thread local-memory pressure — tight caps
are aligned with the device goal, not a compromise.

**Result:** full suite **179/0**; forward substep **0.0026 → 0.0023 ms (~431k/s, +12%)**; gradient path
at parity (fwd ~0.31–0.32 ms, N=21 ~12.6 ms). A small CPU win *and* the hot path is now heap-free.

## 3. Blocker 3 — dynamic topology → flat SoA baked model

**Problem (review §3.3):** the model must be **baked once** into flat SoA constant arrays and uploaded;
the step then reads only per-env mutable state. The diff path already bakes once (`articulationToDiffModel`),
but stores the result in `std::vector<DiffLink>` with a nested `std::vector<ContactSphere>` — not
uploadable.

**Approach — additive, not in-place.** Converting `DiffModel`/`DiffLink` to a POD in place would break
~15 test/header sites that hand-build `md.links = { ... }`, range-iterate `md.links` / `L.contactPoints`,
and call `.size()`. Too risky for the benefit. Instead `DiffModel` stays the **host authoring form**, and
a new **`FlatModel`** is the **device-upload form**, baked from it.

**Change (new `include/engine/physics/diff/flat_model.h`):**
- `struct FlatModel` — fixed-size **SoA** (`parent[]`, `dof[]`, `qIndex[]`, `mass[]`, `Ic[]`, `axes[][3]`,
  `anchorP/C[]`, `restRel[]`, joint props, ground params), `static_assert(std::is_trivially_copyable_v)`
  so it memcpy's to the GPU as one constant blob. `sizeof(FlatModel) = 9232 bytes`.
- `flatten(const DiffModel&) → FlatModel` bakes once and **normalizes** two things the device kernel
  would otherwise branch on: (a) the contact representation — multi-point `contactPoints` ∪ the COM
  `contactRadius` shorthand — into **one flat sphere list** with per-link `[contactOffset, contactCount)`;
  (b) **pre-resolves** `jointDamping` (per-joint override else the inherited global). `kMaxContactSpheres
  = kMaxLinks*8 = 128` (worst case every link a box = 8 corners).
- Constants stay `double` (matching `DiffModel`; the ABA lifts them into `S` via `lift<S>`); the forward
  device kernel casts to float per the precision policy.

**Test (new `tst/physics/unit/diff_flat_model.cpp`, `physics.unit.diff_flat_model`):** bakes humanoid(All)
and AMP(All), verifies **field-for-field fidelity** vs the `DiffModel` (topology, inertia, frames, resolved
damping, and the normalized/flattened contact spheres with correct per-link offset+count), and asserts
trivial-copyability. Humanoid bakes to 14 links / 21 DOF / 64 contact spheres. Full suite **180/0**.

## 4. What's resolved vs. what remains

**Resolved (device-preparatory, CPU-validated):** the ABA hot path is heap-free and fixed-size; the model
has a flat, POD, uploadable form baked once; **and (follow-up below) the per-env state is fixed-size too, so
the entire `diffSubstep` allocates nothing.** All are exercised by the CPU tests, so the exact data layouts
the kernel will use are already validated.

**Remaining CUDA blockers (from the review, not yet addressed):**
- **Virtual `PhysicsWorld` dispatch** (review §3.1) — host-only orchestration; the batched kernel calls a
  concrete `__device__` step. The diff path doesn't use `PhysicsWorld`, so this is a maximal/reduced-backend
  concern, not a diff-kernel one.
- **Precision policy** (§3.4): float forward / double grad-check — the `Dual` scalar and `FlatModel` are
  `double` today; the forward kernel instantiates `float`.
- **Determinism CPU↔GPU** (§3.6): tolerance-based parity tests (FMA/rounding order differs), run on the
  Linux box.
- **Build system** (§3.7): `enable_language(CUDA)` + `-DENGINE_CUDA=ON` gated `if(NOT APPLE)`, a `.cu`
  target, and `Backend::Cuda` in `config.h`.

**Alignment decision still open** (from the state review): the RL env defaults to `Backend::Realtime`
(maximal), but the port targets the reduced/diff dynamics — confirm humanoid RL runs (or switches to) the
reduced + smoothed-contact dynamics so "CPU and CUDA both supported" means the *same* physics on both.

**Next real step:** the batched one-env-per-thread CUDA kernel consuming `FlatModel` + a fixed-size
`DiffState`, validated against this CPU reference — plus the PyTorch-GPU switch (the review's precondition
for the port to pay off end-to-end).

## 3b. Follow-up — per-env state fixed-size: the whole substep is now heap-free (2026-07-08)

The natural continuation of blocker 2, and the last allocation-free piece before the kernel. `DiffState<S>`
held `std::vector<M3<S>> linkRot` + `std::vector<S> qd` (per-env mutable state), and the `SemiImplicit`
(IMEX) path additionally allocated: a trial-state copy, `computeContactForcesWorld` (a `std::vector<V6<S>>`),
and `linkWorld` (four vectors).

**Change (`articulated.h`, + `jacobian.h`, + one test):**
- `DiffState<S>` → fixed-size POD: `M3<S> linkRot[kMaxLinks]`, `S qd[kMaxDof]`, plus `int numLinks/numDof`.
  Now **trivially copyable** — on the GPU it's one struct per thread (or a batched SoA), and the IMEX
  trial-state copy is a plain memcpy. `makeState`/`liftState` set the dims and initialize `linkRot` to
  identity; all external access was already by index, so no ripple beyond one `qd.size()`→`numDof`.
- `computeContactForcesWorld` → returns a fixed-size `ContactForces<S> { V6<S> f[kMaxLinks]; }` and fills it
  via a new heap-free `linkWorldInto(md, st, LinkWorld* out)`. `diffForwardDynamics`'s `extContactWorld` is
  now a plain `const V6<S>*` (into `ContactForces::f`). `linkWorld` keeps a thin `std::vector` wrapper for
  **host-side readout only** (`DiffEnvironment::links()`, energy/COM/observation) — off the hot path.
- `jacobian.h`: the one `DiffState::resize()` site → `makeState<D>(md)`.

**Result:** the entire `diffSubstep` — both the Explicit and IMEX branches — allocates nothing. Suite
**180/0**; substep ~0.0025 ms (parity with baseline), gradient at parity. `DiffState<double>` and
`FlatModel` are now both trivially-copyable POD blobs: the two things a batched kernel uploads (model,
constant) and owns per-env (state, mutable).

## Files changed this session
- `include/engine/physics/diff/articulated.h` — fixed-size ABA + caps (blocker 2); fixed-size `DiffState`,
  `ContactForces`, `linkWorldInto`, pointer `extContactWorld` (§3b)
- `include/engine/physics/diff/flat_model.h` — new `FlatModel` + `flatten()` (blocker 3)
- `include/engine/physics/diff/jacobian.h` — `DiffState::resize()` → `makeState` (§3b)
- `tst/physics/unit/diff_flat_model.cpp` — new fidelity test
- `tst/physics/unit/diff_invariants.cpp` — `qd.size()` → `numDof` (§3b)
