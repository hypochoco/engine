# CUDA port — engine-side next steps (improvements)

Dated (2026-07-08). Engine-only follow-ups from the code review
(`2026-07-08-cuda-port-code-review.md`). These live entirely in `engine::physics` / `engine::physics_env`
— no sim-1 changes (those are deferred; see `2026-07-08-cuda-sim1-integration-next-steps.md`). Ordered
by leverage. The guiding constraint is unchanged: **one implementation for CPU and GPU — never diverge**,
and **never break the CPU/Mac path** (re-run `phys_tests` after each change; CUDA TUs can only be built
on the Linux/NVIDIA box, so device-side changes are compile-verified there, host-side here on the Mac).

## The core decision this implements
Make the **diff ABA + smoothed contact** the ONE RL dynamics on both CPU and GPU. The batched kernel
already runs it; the missing half is a **CPU env that runs the exact same templated code** so that
"CPU and CUDA are the same physics" is literally true (and testable on the Mac). Everything below
serves that.

## E1 — Share the actuator + obs as `ENGINE_HD` free functions (kill the wrapper duplication)
**Problem:** `cuda_vec_env.cu` hand-writes the PD servo (`actionToTauKernel`) and obs packing
(`packObsKernel`) as device kernels — a CPU/GPU divergence risk (the review's MODERATE finding) and the
reason the servo isn't parity-tested.
**Change:** extract the actuation and observation logic into `ENGINE_HD` free functions templated on
`(M, S)`, in a new header `include/engine/physics/diff/actuation.h` (or fold into `articulated.h`):
- `ENGINE_HD void actionToTau(const M& md, const DiffState<S>& st, const S* action, S* tau, int actionMode, S kp, S kd, S maxTorque)` — Torque passthrough+clamp; PD revolute via `revoluteAngle`; PD ball. Move `revoluteAngle`/`ballRotVec`/`matToQuatWXYZ` here as `ENGINE_HD` (they currently live in the `.cu` anonymous namespace).
- `ENGINE_HD void packDefaultObs(const M& md, const DiffState<S>& st, S* obs)` — the exact `Environment::packDefaultObs` layout (root pos/quat wxyz/linVel/angVel | q | qd | contact flags).
Then the CUDA kernels and the new CPU `DiffVecEnv` (E2) BOTH call these — one source of truth. Host-
testable on the Mac via the `double`/`float` instantiation (no CUDA needed).
**Decision to make while doing this:** pin ONE ball-joint PD definition (geodesic/rotation-vector error
is fine) and document it; it replaces both the GPU's component-wise form and any reduced-backend form for
the diff RL path.

## E2 — CPU `DiffVecEnv` (the CPU half of "same physics", fully Mac-testable)
**Problem:** the only CPU RL env today is `physics_env::VecEnv` over the PGS `Backend::Reduced`. There is
no CPU env running the diff ABA the GPU uses.
**Change:** add `engine::physics_env::DiffVecEnv` (host, `include/engine/physics_env/diff_vec_env.h` +
`src/physics_env/diff_vec_env.cpp`) matching the `VecEnv` contract (`numEnvs/actDim/obsDim`,
`actions()/observations()`, `reset(seed)`, `reset_masked`, `step`, `set_articulation_state`). Internals:
a host `std::vector<DiffState<double>>` (or `float` to match the GPU exactly — see note), the shared
`actionToTau`/`packDefaultObs` (E1), and `diffSubstep` per env (optionally over the existing `ThreadPool`
like `VecEnv`). This is the CPU counterpart of `CudaVecEnv` running byte-compatible logic.
- **Precision decision:** default the CPU `DiffVecEnv` to `float` so CPU↔GPU parity is exact-modulo-FMA
  (matches the kernel); keep a `double` mode for reference/debug.
- This immediately gives sim-1 a diff-ABA CPU backend to (re)train on **before** any GPU wiring —
  de-risking the dynamics switch on the machine we can debug on.

## E3 — `CudaVecEnv` feature parity with the CPU env (prereqs for real RL)
**Problem:** `CudaVecEnv` lacks `reset_masked`, `set_articulation_state` (RSI), and seeded per-env init.
**Change (device, compile-verified on the Linux box; keep in lockstep with E2's host logic):**
- `reset_masked(mask, seed)` — a kernel (or `cudaMemcpy` of masked init states) resetting only flagged
  envs' `DiffState` + zeroing their actions, then re-pack obs. Needed for async episode boundaries.
- `set_articulation_state(...)` — upload per-env root pose + joint state into the device `DiffState`
  (RSI / knockdown / domain-randomized init). Mirror the CPU `Environment::setArticulationState`
  reconstruction (base pose + per-joint `linkRot`/`qd`).
- Seeded per-env initial state (domain randomization hook): let `reset(seed)` vary per-env init (the
  device buffer is per-env already).

## E4 — Full-env parity + a shared behavior test (not just bare `diffSubstep`)
**Problem:** parity only covers raw-`tau` `diffSubstep`; the servo/obs/contact-flag paths are untested,
and there's no CPU↔GPU check at the *env* level.
**Change:**
- Host unit test (Mac): `DiffVecEnv` obs layout + PD servo + contact flags vs hand-computed expectations
  and vs the shared free functions — proves E1/E2 correct without a GPU.
- Device parity test (Linux, `ENGINE_CUDA`): `CudaVecEnv` vs `DiffVecEnv<float>` on the same rig/seed/
  actions over K control steps — asserts obs match to float tolerance (the real "same physics" gate),
  extending `cuda_parity.cpp`.

## E5 — Secondary / opportunistic
- **SoA `DiffState` device layout** for coalescing (the buffer is AoS today). Measure occupancy at
  4k/16k/64k first — the step is compute-bound, so this may be marginal; do it only if profiling says so.
- **Config fidelity in `CudaVecEnv`**: honor `DiffContact` selection and the substeps default from
  `SimConfig` rather than hardcoding `DiffContact::All` / `substeps→1`.
- **`Backend::Cuda` semantics**: it's a `CudaVecEnv` selector, not a `PhysicsWorld` — keep that clear (it
  is, in `config.h`), and ensure the factory path documents that CPU-diff uses `DiffVecEnv`, not a
  `PhysicsWorld` backend either. Consider a `Backend::Diff` (CPU diff ABA) enumerator for symmetry so a
  single `EnvConfig` selects reduced-PGS / diff-CPU / diff-CUDA coherently.

## Sequencing for this repo (what to implement now, Mac-verifiable)
1. **E1** (shared `ENGINE_HD` actuator/obs) — host-testable.
2. **E2** (`DiffVecEnv`, CPU) + host tests — the "same physics" CPU half; fully Mac-verifiable.
3. **E3/E4 device pieces** — written in lockstep, compile/parity-verified on the Linux box (cannot build
   on the Mac; `ENGINE_CUDA` is off here).
4. **E5** as profiling dictates.

Verification bar: `phys_tests` (CPU) green after every step; the new `DiffVecEnv` host tests green; the
CUDA build + device parity re-run on the Linux/NVIDIA box (out-of-scope for the Mac).

## Applied (2026-07-08) — what landed in this repo
Implemented E1, E2, E4-host, and the E3 refactor; sim-1 work deferred. Full CPU/Mac suite **181/0**.
- **E1 DONE** — `include/engine/physics/diff/env_ops.h`: shared `ENGINE_HD` `actionToTau<M,S>` +
  `packDefaultObs<M,S>` (+ `revoluteAngle`/`ballRotVec`/`matToQuatWXYZ`/`clampAbs`/`defaultObsDim`),
  the single source for CPU + GPU actuation/obs.
- **E2 DONE** — `include/engine/physics_env/diff_vec_env.h` + `src/physics_env/diff_vec_env.cpp`:
  CPU `DiffVecEnv` (float) over `diff::diffSubstep` + the shared ops; `reset`/`resetMasked`/`step`,
  optional `ThreadPool`. **Reproduces the GPU benchmark's zero-torque humanoid `rootY=0.803` exactly**
  ⇒ the CPU and GPU RL paths now run the same dynamics.
- **E4-host DONE** — `tst/physics_env/integration/diff_vec_env.cpp`: obs dims/layout (69), PD-hold vs
  ragdoll (`yPD=1.163 > yRag=0.803`, actuation works), determinism (bit-identical), `reset_masked`.
- **E3 refactor DONE (device, compile-verified on Linux only — `ENGINE_CUDA` is off on the Mac)** —
  `src/physics_env/cuda/cuda_vec_env.cu` rewritten to call `diff::actionToTau`/`diff::packDefaultObs`
  (deleted the duplicated device kernels' bodies + anon-ns helpers) and adds `resetMasked` (+ device
  init/mask buffers in the header). Realizes E1's single-source benefit on the GPU side too.

**Learned:** stiff PD (`kp=150`) + `float` + smoothed contact is stability-sensitive under actuation
over long horizons (the humanoid NaNs past ~1 s; the GPU path shares this — it only ever validated
~20 control steps). The sim-1 layer manages it (substeps / gains / the divergence guard); engine-side,
the algorithmic lever is the `SemiImplicit` (IMEX) contact to allow fewer substeps (review §7.4).

**Deferred (still open):** `set_articulation_state` RSI on `DiffVecEnv`/`CudaVecEnv` (world-transform →
generalized-state reconstruction — needed for tracking/getup; do with the sim-1 binding when the
signature is fixed); seeded per-env domain-randomization init (both envs currently do deterministic
authored init, matching CPU `VecEnv` without a hook); the **E4 device** parity test (`CudaVecEnv` vs
`DiffVecEnv<float>`, Linux only); E5 SoA layout + `CudaVecEnv` config fidelity.


## E6 — (optional) `CudaVecEnv` step-without-host-download for the zero-copy path

Dated (2026-07-08), from the walk-scoping discussion. **Optional perf micro-opt; not required for a
correct GPU walk pipeline.**

**Context.** For the sim-1 walk integration (deferred sim-1 note, staged D-b), the payoff comes from
keeping obs/actions on-device and wrapping `deviceObservations()/deviceActions()` as torch-CUDA
tensors. Zero-copy already works today with **no engine change** — the consumer just reads `dObs_`
directly. The only inefficiency: `CudaVecEnv::step()` unconditionally runs `packObs()` which *also*
`cudaMemcpy`s the obs batch D→H into the host mirror (`obs_`) every control step. On the zero-copy
path that host download is pure waste (the host mirror is never read).

**Change (small, engine-side, keep CPU/GPU shared-logic intact).** Split the host sync out of `step()`:
- Keep `packObs()` writing the device buffer `dObs_` (the obs kernel), but make the **D→H mirror copy**
  a separate step, e.g. a `syncObservationsToHost()` the host-span `observations()` path calls, or a
  `bool downloadObs = true` parameter / a `stepDevice()` that skips it. Same for `uploadActions()` vs
  a device-actions-already-resident fast path (actions written straight into `dActions_` by the torch
  tensor alias).
- Net effect: the zero-copy `step()` does upload-actions(skip)/kernels/pack-obs(device only) with **no
  PCIe traffic per step**; the span-based contract (`actions()/observations()`) still works by doing
  the copies lazily on access.

**Why it's optional.** Without it the pipeline is correct and still much faster than CPU; you just pay
one redundant `N*obsDim` D→H copy (+ H→D actions) per control step. Measure first (the sim-1 S0/D-b
timing): if the per-step host copy is a negligible fraction of the step wall-time at the target env
count, skip this. If profiling shows it matters, it's a localized change to `cuda_vec_env.{h,cu}` that
does not touch the shared `diffSubstep`/`env_ops` core.

**Verification bar (if done):** `phys_tests` green (CPU unaffected — this is CUDA-only code); the
existing `cuda_vec_env` benchmark unchanged in the download path; add a device-only step timing to show
the saved copy. Keep it behind the same `ENGINE_CUDA` guard.
