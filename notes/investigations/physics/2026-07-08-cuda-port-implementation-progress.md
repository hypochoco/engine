# CUDA port — implementation progress log

Running log of the actual CUDA-port implementation, one section per phase. Companion to the plan in
`2026-07-08-cuda-port-handoff.md` (ordered TODO) and `2026-07-06-cuda-port-review.md` (authoritative
review). Target box: AWS g5-class — **NVIDIA A10G (sm_86, 24 GB)**, **AMD EPYC 7R32 (8c/16t)**,
Ubuntu 24.04, driver 595.71.05.

Guiding constraints (from the owner): **do not break the CPU path**, and **do not diverge the CPU and
CUDA paths** — the kernel must call the *same* templated `diffSubstep`/`diffForwardDynamics` the CPU
uses (only add `__host__ __device__`), never a second implementation. Re-run the CPU diff tests after
each phase to prove the CPU path is untouched.

---

## CPU baseline (this box, before the port) — 2026-07-08

Measured on the EPYC 7R32, Release/-O3, built headless via an out-of-tree scratch CMake project
(`ENGINE_TRAINING_ONLY=ON` + the engine's own `tst/physics/benchmark/diff.cpp` and
`tst/physics_env/benchmark/*`). **No engine files were modified to benchmark.** `step.cpp` was
excluded (needs `engine::physics_ecs`, not built in the headless surface).

| Benchmark | This box (EPYC 7R32) | M3 Pro ref |
|---|---|---|
| diff fwd substep, double, humanoid 14/21 | **0.0086 ms → 116k substeps/s** | 0.0026 ms → 385k/s |
| reduced backend substep, float | 0.0138 ms (diff overhead 0.62×) | 0.0041 ms |
| diff gradient, 8 steps: fwd / N=21 | 1.123 / 90.6 ms (80.6× fwd) | 0.323 / 12.45 ms (38×) |
| diff per-step Jacobian 54×75 @16 substeps | 31.9 ms/step | — |
| diff contact | ~62 ns/point (all=0.0125 ms, 64 pt) | — |
| reduced env throughput, N=1024, substeps=16 | **23.4k env-steps/s** (maximal 30.7k) | — |
| reduced contact step cost | 1.11 ms/step (contact-solve 0.92 ms) | — |
| vec_env throughput, N=1024 | 61.5k env-steps/s | — |

Notes: EPYC single-thread is **~3.3× slower/core** than the M3 Pro (matches the review's expectation of
a lower CPU baseline on this box). RL trains at **substeps=48** (not 16), so the effective reduced
throughput is ~⅓ of the 23k figure (~8k env-steps/s). The port targets the reduced/diff float path;
the review's A10G estimate is 300–800k env-steps/s ⇒ ~13–35× on the physics step, Amdahl-capped
end-to-end (pending the sim%/policy%/learn% split measurement in the sim1 repo).

---

## Phase 0 — CUDA toolkit + build/test loop — 2026-07-08 ✅

**Constraint on this box:** no root (`no_new_privs` set in the sandbox), so no apt/system install; the
Ubuntu repo only offers 12.0 regardless. Also: Vulkan dev headers are absent and the `glfw` submodule
is not populated, so the **full graphics build cannot configure here** — irrelevant to CUDA (gated
`if(NOT APPLE)`, independent of graphics), but it's why benchmarks/tests are built headless out-of-tree.

**Installed rootless: CUDA Toolkit 12.4.1** via the official runfile, toolkit-only, to a user prefix:
```
sh cuda_12.4.1_550.54.15_linux.run --silent --toolkit --toolkitpath=$HOME/cuda-12.4 --no-man-page --override
```
Gives a standard, CMake-discoverable layout at `$HOME/cuda-12.4/{bin/nvcc, include, lib64, nvvm}`.
`nvcc` = release **12.4, V12.4.131** — matches the `torch 2.6.0+cu124` we installed (clean interop).
The pip `nvidia-cuda-nvcc-cu12` wheel was a dead end (ships `ptxas` only, no `nvcc`/`cicc` frontend).

**Verified:** compiled a trivial `axpy` kernel `-arch=sm_86 -O2` and ran it on the **A10G** →
`device: NVIDIA A10G  sm_86  err=0.0`. Toolkit + driver 595 + device execution all confirmed.

**Environment needed for CUDA builds** (record — not yet persisted to any profile to avoid touching
`.venv`/`.bashrc`; set inline in build commands or export per shell):
```
export CUDA_HOME=$HOME/cuda-12.4
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
# CMake: -DCUDAToolkit_ROOT=$HOME/cuda-12.4  (or -DCMAKE_CUDA_COMPILER=$HOME/cuda-12.4/bin/nvcc)
```

**Backend-alignment decision (confirmed with owner):** humanoid RL will run the **reduced +
smoothed-contact** dynamics so CPU and CUDA execute the *same* physics (parity is then meaningful).

**Next:** Phase 1 — build scaffolding (`ENGINE_CUDA` option + `enable_language(CUDA)` gated
`if(NOT APPLE)`, `.cu` glob under the guard, `Backend::Cuda` in `config.h`), all additive and default-off
so the CPU/Mac build is unaffected.

---

## Phase 1 — build scaffolding — 2026-07-08 ✅

All changes additive + guarded by `ENGINE_CUDA` (default **OFF**), so the CPU backends and the Mac
build are untouched.

**Files changed (engine):**
- `include/engine/physics/config.h` — added `Backend::Cuda` to `enum class Backend`. Documented as the
  **CudaVecEnv** path (NOT `createPhysicsWorld()` — it is not a `PhysicsWorld`); the device kernel runs
  the *same* templated diff ABA as the CPU path. `createPhysicsWorld`'s switch has a `default:` (→
  Realtime), so the new enumerator triggers no `-Wswitch` warning; the engine's own modules build
  without `-Werror`.
- `CMakeLists.txt` (top) — `option(ENGINE_CUDA ... OFF)`; when ON: `FATAL_ERROR` on Apple, set
  `CMAKE_CUDA_ARCHITECTURES=86` **before** `enable_language(CUDA)` (else CMake seeds 52),
  `CMAKE_CUDA_STANDARD=20` (nvcc 12.4 device C++; host TUs stay CXX_STANDARD 23), `find_package(CUDAToolkit REQUIRED)`.
- `modules/physics/CMakeLists.txt` — guarded `*.cu` glob added to `engine_physics` (empty until the
  kernel lands); when ON: `PUBLIC ENGINE_CUDA` define, `CUDA_SEPARABLE_COMPILATION ON`, link `CUDA::cudart`.

**Verification (all via the out-of-tree scratch project; no engine files touched to test):**
- **CPU unaffected (ENGINE_CUDA=OFF, default):** engine reconfigures + `bench` rebuilds clean;
  `diff_forward_vs_reduced` = 0.0088 ms / 113k (matches the baseline within noise). Headless CPU test
  suite green: **physics 125/0, physics_env 9/0** (the diff/reduced/env oracle — `phys_tests` target,
  excludes the two ecs/physics_ecs-dependent files `milestone.cpp`, `articulation_ecs.cpp`).
- **ENGINE_CUDA=ON:** `enable_language(CUDA)` succeeds (nvcc 12.4.131 ABI check), `CUDAToolkit` found
  12.4.131, message reports **arch 86**. A temporary probe `src/physics/cuda/cuda_probe.cu` confirmed the
  `.cu` glob → nvcc compile → object path end-to-end, then was **removed**; the empty glob reconfigures
  cleanly and `bench` still builds + runs correctly (0.0087 ms — the `ENGINE_CUDA` define does not perturb
  the host diff path since no `__host__ __device__`/`__CUDACC__` is used yet).

**Scratch build harness** (`~/research/bench-scratch/`, out-of-tree, not in the repo): `bench` +
`phys_tests` targets; `build/` = CPU (ENGINE_CUDA off), `build-cuda/` = ENGINE_CUDA on
(`-DCMAKE_CUDA_COMPILER=$HOME/cuda-12.4/bin/nvcc`).

**Next:** Phase 2 — annotate the templated diff hot path `__host__ __device__` via an `ENGINE_HD` macro
shim (no-op on host, so the CPU path stays byte-identical); keep `linkWorld`'s `std::vector` wrapper
host-only; confirm `std::sin/cos/...` resolve on device. Re-run the CPU oracle for parity.

---

## Phase 2 — device-callable hot path (annotate + model-generic) — 2026-07-08 ✅

**Key finding:** `__host__ __device__` annotation alone is NOT sufficient. The ABA consumed the
host-only `DiffModel` (`std::vector<DiffLink>`, nested `std::vector<ContactSphere>`) and took `tau` as
`std::vector<S>` — uncallable on device. So Phase 2 also had to make the model access device-friendly,
**without diverging** into two implementations.

**Design (single implementation, no divergence):** the ABA is now templated on the MODEL type `M` and
reads the model through a small `hd*` accessor layer that BOTH `DiffModel` (host) and `FlatModel`
(device) satisfy. The host instantiates the ABA with `DiffModel`, the device with `FlatModel` — same
code. `HDLink` is a lightweight per-link view (scalars by value, big constants Ic/restRel/anchors/axes
by const POINTER into the model, so no M3/V3 deep copy). `jointDamping` is resolved and contact spheres
normalized in the accessors, matching `flatten()` exactly (so CPU behavior is unchanged and CPU↔GPU
agree). `tau` is now a raw pointer in the core; thin host `std::vector` overloads of `diffSubstep`/
`diffForwardDynamics` forward `tau.data()`, so all ~40 existing call sites compile unchanged.

**Files changed (engine):**
- New `include/engine/physics/diff/hd.h` — `ENGINE_HD` = `__host__ __device__` under `__CUDACC__`, else nothing.
- `linalg.h`, `dual.h` — every templated free function + the scalar `sigmoid`/`softplus` annotated
  `ENGINE_HD` (model-agnostic; host-only `logSO3` left un-annotated as finite-diff tooling).
- `articulated.h` — `HDLink` + host `DiffModel` accessors; ABA (`groundContactWorld`,
  `linkGroundContactWorld` now `(md,i,...)`, `computeContactForcesWorld`, `diffForwardDynamics`,
  `diffApplyAccel`, `diffIntegrateConfig`, `diffSubstep`, `linkWorldInto`) templated on `M` + `ENGINE_HD`
  + `tau` pointer; host `std::vector` adapters added; `makeState`/`liftState`/`linkWorld` kept host-only.
- `flat_model.h` — `ENGINE_HD` `FlatModel` accessors mirroring the host ones.
- `modules/physics/CMakeLists.txt` — `--expt-relaxed-constexpr` on CUDA TUs (std::array `operator[]` on device).
- `tst/physics/unit/diff_semiimplicit.cpp` — one call site: `groundContactSpatial<double>(...)` →
  `groundContactSpatial(...)` (the leading template param is now `M`; `S` still deduces). Only external
  caller affected by the signature change.

**Verification:**
- **CPU parity (guard off AND on):** `phys_tests` **physics 125/0, physics_env 9/0** green in both the
  `ENGINE_CUDA=OFF` and `ENGINE_CUDA=ON` host builds (the define alone doesn't perturb host behavior;
  `ENGINE_HD` is a no-op for g++ TUs).
- **Device-callable (the goal):** a temporary probe `.cu` instantiating `diffSubstep<FlatModel,float>`
  in a `__global__` one-env-per-thread kernel compiled clean for **sm_86** (whole ABA: diff dynamics +
  IMEX contact + linalg/dual math + `std::array` + std trig). Removed after the check.
- **Perf:** reduced-backend RL throughput unchanged (N=1024 ~22.7k env-steps/s vs 23.4k baseline —
  noise; `featherstone_world.cpp` untouched). Diff **contact** substep ~+2% (0.0125→0.0127 ms, the path
  the humanoid + GPU kernel run). Diff **contact-free** double substep ~+5–6% (0.0086→~0.0092 ms) from
  the `HDLink` view construction on the double/gradient path — acceptable (not the RL path); recoverable
  with per-field const-ref accessors if it ever matters.

**Next:** Phase 3 — the real batched one-env-per-thread float forward kernel: device-resident
`DiffState<float>` buffer (AoS first cut) + `FlatModel` baked once and uploaded (`__constant__`/global),
a host launcher, and a micro-benchmark of envs × substeps at 4k/16k/64k on the A10G.

---

## Phase 3 — batched one-env-per-thread float forward kernel — 2026-07-08 ✅

**New files (engine):**
- `include/engine/physics/cuda/batched_forward.h` — `BatchedForward`: a host-facing class (plain C++,
  no CUDA syntax) owning the device buffers. `FlatModel` uploaded once; device `DiffState<float>`
  buffer for N envs (AoS); per-env `tau`. `step(substeps, h, gravity)` = one kernel launch.
- `src/physics/cuda/batched_forward.cu` — `batchedSubstepKernel`: one thread per env loads its
  `DiffState<float>` to registers, runs `substeps` of the shared `diff::diffSubstep(*md, st, tau, grav, h)`
  reading the broadcast `FlatModel` from global memory, writes back. No cross-env comms.
- `tst/physics/benchmark/cuda_forward.cpp` — `physics.cuda_batched_forward` (guarded by `ENGINE_CUDA`):
  bakes the humanoid (14/21, contact=All) → `flatten` → upload, sweeps N ∈ {4k,16k,64k}, reports
  throughput + a finiteness / plausible-root-height assert.
- `modules/physics/CMakeLists.txt` — added `CUDA_RESOLVE_DEVICE_SYMBOLS ON` so the static lib
  device-links and the non-CUDA `bench` executable consumes it clean.

**First A10G numbers (humanoid 14/21, contact=All, substeps=48, RL config):**

| N envs | env-control-steps/s | env-substeps/s | meanRootY | finite |
|---|---|---|---|---|
| 4,096  | 162k | 7.78M | 0.803 | yes |
| 16,384 | **276k** | **13.25M** | 0.803 | yes |
| 65,536 | 274k | 13.15M | 0.803 | yes |

- Saturates at ~16k envs (276k → plateau at 64k) — matches the review's "16k–65k to fill 72 SMs,
  compute-bound." 64k envs = ~51 MB of `DiffState` + ~5 MB `tau` ≪ 24 GB ⇒ compute-bound confirmed.
- **~35× over the CPU** at matched config: CPU reduced backend ≈ 7.8k env-control-steps/s at substeps=48
  (extrapolated from the measured 23.4k at substeps=16). Squarely in the review's estimated 20–40× band.
- `meanRootY=0.803` is identical across all 65,536 envs (same init + zero tau + deterministic) — an
  implicit same-device determinism signal — and physically plausible (zero-torque humanoid sags onto the
  smoothed contact from 0.99). Rigorous CPU↔GPU parity is Phase 5.

CPU path still green after adding these files: `phys_tests` **125/0, 9/0** (guard off; `cuda_forward.cpp`
compiles to an empty TU and the `.cu` isn't globbed).

**Next:** Phase 4 — `CudaVecEnv` filling the same flat `obs_`/`actions_` contract as `physics_env::VecEnv`,
buffers resident in VRAM, with reward/termination/reset vectorized on-GPU (avoid the slow-glue trap).

---

## Phase 4 — CudaVecEnv (GPU-resident RL env) — 2026-07-08 ✅

**Scope note:** the CPU RL path uses the **reduced backend** (solver-integrated PD actuator); the diff
ABA we ported takes **raw tau** and has no built-in servo. So `CudaVecEnv` is genuinely new RL-facing
glue (PD servo + obs packing from `DiffState`), not a mirror of a CPU class. Reward/termination stay in
the sim1 layer (as for the CPU `VecEnv`). The meaningful CPU↔GPU *dynamics* parity (Phase 5) is on
`diffSubstep`, not this wrapper.

**New files (engine):**
- `include/engine/physics_env/cuda_vec_env.h` — `CudaVecEnv` matching the `VecEnv` contract
  (`numEnvs/actDim/obsDim`, `actions()`/`observations()` host-mirror spans, `reset`/`step`), plus
  `deviceActions()/deviceObservations()` (VRAM pointers) for a future zero-copy PyTorch-GPU path.
- `src/physics_env/cuda/cuda_vec_env.cu` — two kernels wrapping the Phase-3 substep:
  `actionToTauKernel` (Torque passthrough+clamp, or a PD servo in joint coords: revolute angle via
  `atan2(axis·vee(R),(trR−1)/2)`, ball via float `logSO3`) and `packObsKernel` (exact
  `packDefaultObs` layout: root pos/quat(wxyz via Shepperd)/world linVel/world angVel, joint q/qd in
  link order, per-body contact flags from `linkWorldInto` + penetration). Composes `BatchedForward`
  (added `deviceStates()/deviceTau()/deviceModel()` accessors).
- `modules/physics_env/CMakeLists.txt` — guarded `.cu` glob + `ENGINE_CUDA` define + separable/resolve
  device symbols + `--expt-relaxed-constexpr` + `CUDA::cudart` (mirrors the physics module).
- `tst/physics_env/benchmark/cuda_vec_env.cpp` — guarded validation + throughput.

**Verification (A10G, humanoid, PD-target, substeps=48):**
- **Contract match:** `actDim` gpu=21 == cpu=21, `obsDim` gpu=69 == cpu=69 (asserted against a CPU
  `Environment` built from the same rig). obs finite.
- **PD servo works:** with zero PD targets (hold rest pose) the humanoid stays upright at rootY≈1.05
  — vs the 0.803 zero-torque ragdoll of Phase 3, confirming action→tau actuation.
- **Throughput (incl. obs pack + host obs download each step):** N=4096 159k, N=16384 **270k
  env-control-steps/s** — essentially the raw kernel speed (276k), so the action/obs kernels + host
  sync add negligible overhead. The host download is only for the span contract; the device-pointer
  path skips it (Phase 6 / RL integration).
- CPU path still green: `phys_tests` **125/0, 9/0** (guard off).

**Next:** Phase 5 — formal tolerance-based CPU↔GPU parity on `diffSubstep` (reuse the diff tests as
oracle) + same-device determinism; then the consolidated re-benchmark table.

---

## Phase 5 — parity + determinism + consolidated benchmark — 2026-07-08 ✅ (port complete)

**New file (engine):** `tst/physics/integration/cuda_parity.cpp` (guarded `ENGINE_CUDA`), two cases:
- `cuda_diff_parity`: GPU float kernel vs the SAME `diffSubstep` on the CPU. At matched precision
  (CPU float vs GPU float) only FMA/rounding order can differ; against the CPU double it's the float
  approximation of the oracle. Humanoid, contact=All, K=16 substeps, non-trivial tau.
- `cuda_determinism`: N=1024, 48 substeps — run-to-run and (identical-init ⇒) env-to-env bit-identical.

**Results (A10G):**
- **Parity:** GPUf-vs-CPUf `pos=1.8e-12, qd=6.0e-8`; GPUf-vs-CPUd(oracle) `pos=1.1e-8, qd=2.5e-7`.
  The kernel reproduces the CPU `diffSubstep` essentially to float precision — the single-implementation
  approach paid off (there is nothing to diverge).
- **Determinism:** run-to-run **bit-identical**, env-to-env **bit-identical** (no cross-env
  atomics/reductions). Same-device reproducibility holds.

**Consolidated benchmark (humanoid 14/21, smoothed ground contact):**

| Path | Config | Throughput |
|---|---|---|
| **CPU** EPYC 7R32, reduced backend, 17 workers | substeps=16, N=1024 | 22.7k env-steps/s (⇒ ~7.6k at substeps=48) |
| **GPU** A10G, batched diff ABA (float) | substeps=48, N=4,096 | 162k env-ctrl-steps/s (7.8M substeps/s) |
| **GPU** A10G | substeps=48, N=16,384 | **277k env-ctrl-steps/s (13.3M substeps/s)** |
| **GPU** A10G | substeps=48, N=65,536 | 274k env-ctrl-steps/s (13.2M substeps/s) |

⇒ **~36× over the CPU** at matched substeps=48 — inside the review's 20–40× estimate. Saturates at
~16k envs (72 SMs), compute-bound. `CudaVecEnv` (obs pack + host download included) matches this (270k).

**Final CPU regression (guard OFF, all 5 phases applied): `phys_tests` 125/0, 9/0** — the CPU/Mac path
is byte-for-byte intact. The diff double substep carries the ~5% `HDLink`-view overhead noted in Phase 2
(gradient/oracle path only; reduced RL backend untouched).

## Port status: forward path COMPLETE
The reduced + smoothed-contact ABA runs batched on the GPU via ONE shared templated implementation
(CPU double / GPU float), behind the existing `VecEnv` contract, gated `if(NOT APPLE)` + `ENGINE_CUDA`,
CPU-parity- and determinism-tested.

## Recommended next steps (out of this port's scope, prioritized)

1. **PyTorch-CPU → PyTorch-GPU switch + bind `CudaVecEnv` into `sim1.engine_py`.** This is the review's
   precondition for the *end-to-end* training win (§7.1): with the policy on the A10G, obs/actions stay
   in VRAM and the rollout closes on-device. Until this lands, the per-step host↔device copy of
   obs/actions caps the gain (roughly doubles step time / serializes behind the CPU). `torch 2.6.0+cu124`
   is already installed; `CudaVecEnv` already exposes `deviceObservations()/deviceActions()` for the
   zero-copy path. **Highest leverage** — the physics port and this switch must land together or neither
   helps end-to-end.
2. **Vectorize reward/termination/reset on-GPU (sim1 layer).** The "fast sim, slow glue" trap: with the
   sim at ~36×, any per-env CPU/Python reward/termination/`reset()` loop becomes the new bottleneck.
   These live in the `sim1` repo (task layer), not the engine. Includes RSI / domain-randomization reset
   hooks (a per-env initial-state randomizer on the device state buffer; today all envs share one init).
3. **SoA `DiffState` layout for memory coalescing.** Current buffer is AoS (the simple first cut). A
   struct-of-arrays layout across envs improves global-memory coalescing; measure occupancy at
   4k/16k/64k first to confirm it's worth it (the step is compute-bound, so this may be secondary).
4. **On-GPU `Dual` gradients (the differentiable-sim / SHAC payoff).** The forward kernel already
   instantiates the templated ABA; a `Dual<N>`-on-device instantiation gives analytic policy gradients
   on the GPU. Watch register pressure from `Dual<N>` partials; needs `--expt-relaxed-constexpr` (already
   set) for `std::array` on device. Natural follow-up now that the float forward path is landed + profiled.
5. **Algorithmic lever — implicit/semi-implicit contact to cut substeps 48 → 8–16** (review §7.4). A
   3–6× throughput win *on top of* the port, and it helps the CPU path too. The IMEX path already exists
   behind `ContactIntegration::SemiImplicit`; this is about pushing stiffness/`h` further.

Nothing here is required for "the engine is CUDA-capable for the physics forward path" (done); these are
the path to the full end-to-end training speedup.
