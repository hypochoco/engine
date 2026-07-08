# CUDA port — handoff / TODO

**Audience:** the engineer picking up the physics CUDA port. **Status (2026-07-08):** CPU-side
device-prep is done; no `.cu` exists yet. This doc is the ordered work plan. Read it top to bottom.

## 0. Context in 60 seconds

- **Goal:** run the physics sim on the GPU **alongside** the CPU backends — CPU+Metal on the Mac dev
  machine, CPU+CUDA on a Linux/NVIDIA box — from one codebase. CUDA is gated `if(NOT APPLE)`; the Mac
  never builds it.
- **What to port:** the **reduced + smoothed-contact ABA**, batched **one-env-per-thread** (NOT the
  maximal solver, NOT intra-world parallelism). This matches the parallel-worlds RL design and MJX/Brax.
- **The code you port lives in** `include/engine/physics/diff/` (scalar-generic templated ABA). It is
  already CPU-validated, heap-free, fixed-size, and POD — see §1.
- **Background reading (in order):**
  1. `notes/investigations/physics/2026-07-06-cuda-port-review.md` — the full review (target HW, what
     ports well, perf estimates). **Authoritative.**
  2. `notes/investigations/physics/2026-07-08-cuda-port-blockers-fixed-size-flat-model.md` — the CPU
     prep already done (fixed-size ABA, `FlatModel`, fixed-size `DiffState`).
  3. `notes/investigations/physics/2026-07-04-differentiable-reduced.md` — the differentiable ABA design.

## 1. What's already done for you (CPU-side, all tests green: 180/0)

- **`diffForwardDynamics` is heap-free** — fixed-size stack arrays sized by `kMaxLinks=16`/`kMaxDof=32`
  (`articulated.h`). Bump the caps for bigger rigs (both built-in rigs fit: humanoid 14/21, AMP 15/28).
- **`DiffState<S>` is a fixed-size, trivially-copyable POD** (`linkRot[kMaxLinks]`, `qd[kMaxDof]`,
  `numLinks/numDof`). The **entire `diffSubstep`** (Explicit + IMEX) allocates nothing.
- **`FlatModel` (`flat_model.h`)** — the model constants baked once into a flat, POD, `is_trivially_copyable`
  SoA blob (`sizeof ≈ 9.2 KB`), via `flatten(const DiffModel&)`. Contact reps normalized, joint damping
  pre-resolved. Fidelity-tested (`physics.unit.diff_flat_model`).

⇒ The two objects the kernel needs are already POD: **`FlatModel`** (constant, upload once) and
**`DiffState`** (mutable, one per env). The CPU `diffSubstep` is the reference the kernel must match.

## 2. FIRST: benchmark / measure before writing any `.cu`

The port only pays off if the sim is a large share of training wall-time and the whole loop is on-GPU.
Get these numbers first — they set the ceiling and the saturation target.

1. **CPU baseline (already have it, on Apple M3 Pro — re-run on the actual CPU you compare against):**
   ```
   cmake --build build-release -j --target benchmarks
   ./build-release/tst/benchmarks physics.diff_forward_vs_reduced   # single-env substep cost
   ./build-release/tst/benchmarks physics.diff_gradient_cost        # forward-mode grad scaling
   ./build-release/tst/benchmarks physics.step                      # maximal backend + parallel worlds
   ```
   M3 Pro reference: diff forward substep **0.0025 ms → ~395k substeps/s** (single-thread, double);
   parallel-worlds 7.31× on 12 workers. On the A10G box's 8-core EPYC expect a **lower** CPU baseline.
2. **Amdahl split (DO THIS — it's the highest-leverage number, and it's missing).** Profile the downstream
   `sim1` PPO loop and record **sim % / policy-forward % / learn %** of wall-time. If sim is only ~50%
   today, even a perfect kernel caps end-to-end speedup at ~2×. This decides whether the port is worth it
   and whether the PyTorch-GPU switch must land first. Lives in the RL repo, not here.
3. **After a prototype kernel:** micro-benchmark `envs × substep` on the A10G at 4k / 16k / 64k envs to
   find the SM-saturation point and real env-steps/s. Review §7.2 estimates ~300k–800k env-steps/s and
   wants ~16k–65k envs to saturate 72 SMs (compute-bound, not memory-bound).

Expected wins (estimates, review §7): ~20–40× on the physics step, ~10–30× end-to-end (Amdahl). The
**48 substeps/control-step** is the biggest algorithmic lever — a semi-implicit/implicit contact solve
that drops it to 8–16 is a 3–6× win on top of the port and helps the CPU too (IMEX already exists in the
diff code behind `ContactIntegration::SemiImplicit`).

## 3. THEN: features left to implement (ordered)

### 3.1 Build-system scaffolding (do first, small)
- [ ] `enable_language(CUDA)` + `option(ENGINE_CUDA ...)`, gated `if(NOT APPLE)`, in the top `CMakeLists.txt`.
- [ ] A CUDA target/subtree (separable compilation). `modules/physics/CMakeLists.txt` currently globs
      `*.cpp`; add a `.cu` glob under the `ENGINE_CUDA` guard.
- [ ] Add `Backend::Cuda` to `include/engine/physics/config.h` (`enum class Backend`).
- [ ] Stand up the **remote build/test loop on the Linux/NVIDIA box** immediately — the Mac can't build or
      run any of this. This is the biggest non-technical risk; do it before writing kernel logic.

### 3.2 The batched forward kernel (the core deliverable)
- [ ] Make the diff hot functions callable from device code: annotate the templated ABA path
      (`diffForwardDynamics`, `diffSubstep`, `diffApplyAccel`, `diffIntegrateConfig`, `linkWorldInto`,
      `groundContactWorld` & friends, and the `linalg.h`/`dual.h` operators) `__host__ __device__`.
      They're already header-only, templated, fixed-size, branch-light — this should be mostly mechanical.
      Watch: `std::sin/cos/sqrt/exp/log1p` → CUDA device intrinsics (they resolve, but confirm); no
      `std::vector` remains in the step (done); no virtual dispatch in the diff path (there is none).
- [ ] A kernel: one thread (or warp) per env → each calls the concrete `__device__` `diffSubstep` for
      `substeps`, reading the shared `FlatModel` (constant/`__constant__` or global) and its own
      `DiffState` from a batched buffer.
- [ ] Precision: instantiate the forward kernel as **`float`** (`Real`-style). `Dual`/`FlatModel` are
      `double` today; keep `double` only for offline grad-checks (A10G FP64 is 1/32 rate).
- [ ] Register/local-memory pressure is the main perf risk (the per-substep working set is
      `O(kMaxLinks)` spatial matrices). Tight caps already help; profile occupancy at 4k/16k/64k envs.

### 3.3 Batched state buffer + `CudaVecEnv`
- [ ] A device-resident batched `DiffState` buffer (SoA across envs is ideal for coalescing; AoS of the
      POD `DiffState` is the simple first cut). Bake `FlatModel` once, upload once.
- [ ] `CudaVecEnv` fills the **same** flat `obs_`/`actions_` contract as `physics_env::VecEnv` — keep the
      RL-facing API identical. Buffers stay in **device memory**; cross to host only if the policy is on CPU.
- [ ] Vectorize the "glue" on-GPU too: reward/termination/`reset()` (`environment.cpp`
      `applyInitialState`/`updateContactFlags` + downstream reward/curriculum in the `sim1` repo). Any
      per-env CPU/Python loop here becomes the new bottleneck (the classic "fast sim, slow glue" trap).

### 3.4 Parity + determinism tests (on the Linux box)
- [ ] CPU↔GPU parity: run the same rig+seed on both, assert per-step state agreement to a **tolerance**
      (NOT bit-exact — FMA/rounding order differs). Reuse the diff tests as the oracle
      (`tst/physics/{unit/diff_*, integration/diff_*}`).
- [ ] Determinism: per-env independence keeps each env reproducible **same-device**; add a "same-device"
      qualifier to determinism tests. No cross-env atomics/reductions in the step.

### 3.5 The precondition for the payoff (coordinate with the RL team)
- [ ] **PyTorch-CPU → PyTorch-GPU switch** so obs/actions never leave VRAM. Per review §7.1, a per-step
      round-trip to a CPU policy roughly doubles step time — the physics port and this switch must land
      together or neither helps.

## 4. Explicitly OUT of scope (punt)
- The **maximal-coordinate `sequential_impulse_world.cpp`** on GPU (Gauss-Seidel is serial within a world;
  needs graph-coloring/Jacobi). Not needed for humanoid RL.
- **General GPU collision** (GJK/EPA/SAP — divergent/iterative). The smoothed analytic ground contact
  covers the humanoid-on-plane RL task; defer arbitrary convex-convex on device until actually needed.
- **On-GPU gradients** (`Dual` on device) — a natural extension once the forward kernel exists and a real
  win for the diff-research track, but not required for RL throughput. Register pressure from `Dual<N>`
  partials is the risk. Do it after the float forward kernel is landed and profiled.

## 5. One decision to confirm before starting
The RL env defaults to `Backend::Realtime` (maximal), but the port targets the **reduced/diff** dynamics.
Confirm humanoid RL training runs (or switches to) the reduced + smoothed-contact dynamics — otherwise
"CPU and CUDA both supported" would mean two *different* contact models, and CPU↔GPU parity is meaningless.

## Key files
- `include/engine/physics/diff/articulated.h` — the ABA to port (`diffSubstep`, `diffForwardDynamics`, …)
- `include/engine/physics/diff/flat_model.h` — `FlatModel` + `flatten()` (the upload blob)
- `include/engine/physics/diff/{linalg,dual}.h` — the device math substrate (annotate `__host__ __device__`)
- `include/engine/physics/diff/from_articulation.h` — `ArticulationDef → DiffModel` (then `flatten`)
- `include/engine/physics/config.h` — add `Backend::Cuda`
- `include/engine/physics_env/vec_env.{h,cpp}` — the flat obs/action contract `CudaVecEnv` must match
- `tst/physics/{unit,integration}/diff_*.cpp` — the CPU reference/oracle for parity tests
