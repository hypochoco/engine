# CUDA port — code review & integration assessment (returned handoff)

Dated (2026-07-08). A deep-dive review of the CUDA physics port after it came back from the handoff
(`sim-1-cuda`: the `sim-1` RL repo with the engine as `external/engine`). Companion to the
implementation log (`2026-07-08-cuda-port-implementation-progress.md`, the author's phase-by-phase
account) and the plan (`2026-07-08-cuda-port-handoff.md`). This note is the **independent review**:
what was verified against the actual code, what holds up, and the gaps that shape the next steps
(engine-side: `2026-07-08-cuda-engine-next-steps.md`; sim-1-side:
`2026-07-08-cuda-sim1-integration-next-steps.md`).

**Method:** read the merged engine code (`diff/{hd,articulated,flat_model,linalg,dual}.h`,
`physics/cuda/batched_forward.*`, `physics_env/cuda_vec_env.*`, the CUDA tests, the CMake gating) and
the sim-1 side (`csrc/engine_py.cpp`, `sim1/envs/engine_vecenv.py`, `HANDOFF.md`). Findings below cite
what was read; where I relied on a note rather than reading a file, it's called out.

## Verdict in one paragraph
The **engine-level port is high quality** and its hardest goal is genuinely met: a SINGLE templated ABA
(`diffSubstep<M,S>`) runs on both CPU and GPU with no divergence, the CPU/Mac path is byte-for-byte
intact (`phys_tests` 125/0, 9/0 with the guard off *and* on), and GPU↔CPU parity is tight
(`pos=1.8e-12` float-vs-float). But (1) there is a **contact-model / backend mismatch** that means the
GPU does not run the same physics the sim-1 trainer actually uses, (2) the GPU env is **not wired into
sim-1** and is **missing RL-critical features** (per-env reset, RSI), and (3) the RL-facing wrapper
**reintroduces CPU/GPU code duplication** the core ABA had carefully avoided. Treat the deliverable as
"the GPU forward kernel exists and is correct," **not** "training is CUDA-accelerated."

## What was built (and what's genuinely excellent)
Five phases, all in the engine under `ENGINE_CUDA` (default OFF, `if(NOT APPLE)`):
- **Model-generic ABA (the key design win).** `diffSubstep`/`diffForwardDynamics`/… are templated on the
  MODEL type `M` and read it through `hd*` accessors + a light `HDLink` view (scalars by value, big
  constants Ic/restRel/anchors/axes by const pointer). Annotated `ENGINE_HD` (`__host__ __device__`
  under nvcc, **no-op on host**, via `hd.h`). Host instantiates with `DiffModel`, device with
  `FlatModel` — **one implementation**. Thin `std::vector` host adapters keep all ~40 CPU call sites
  compiling unchanged. `linalg.h`/`dual.h` are annotated the same way. This is exactly the right shape
  and is cleanly executed.
- `BatchedForward` — one-thread-per-env kernel; `FlatModel` uploaded once; `DiffState<float>` buffer
  (AoS). `CudaVecEnv` — action→tau + obs-pack kernels over it. Parity + determinism tests.

**Verified strengths:** CPU regression green throughout; `FlatModel` is a faithful `is_trivially_copyable`
blob; the build gating is additive and default-off (I diffed every modified file — the only changes are
`ENGINE_HD` annotations, `Backend::Cuda`, the model-generic templating, and guarded CMake; no behavioral
edits to the CPU path). The implementation log is honest and specific. **The hardest part is done well.**

## Findings, by severity

### 🔴 CRITICAL — the GPU and the sim-1 trainer run different contact models
- sim-1 trains through `engine_py.VecEnv` → CPU `Environment` → **`Backend::Reduced`** = the Featherstone
  backend with **hard PGS contact** (Baumgarte + slop + sequential impulses — confirmed in
  `featherstone_world.cpp`: "generalized-coordinate PGS", `reducedBaumgarte`/`reducedSlop`, `pgsIterations`).
- The GPU `CudaVecEnv` runs the **differentiable ABA** with **smoothed compliant contact** (softplus
  spring + gated damping penalty; penetration allowed) — a fundamentally different contact model.
- The Phase-5 parity test validates **GPU-diff-float vs CPU-diff-double** — the GPU against the *diff*
  path, which sim-1 does **not** use. Nothing validates GPU vs the reduced/PGS backend the trainer runs.
- Contact dominates a standing/walking humanoid's dynamics. A policy trained on PGS contact will not
  transfer faithfully to smoothed-penalty contact, so "run both on Mac and Linux, same behavior" is
  **not** achieved by the current design.
- **Root cause — a terminology trap.** The 2026-07-06 review said "port the *reduced + smoothed-contact
  ABA*," where "reduced" meant the *reduced-COORDINATE differentiable ABA* (which has smoothed contact).
  The Phase-0 alignment decision recorded it as "RL will run the *reduced* dynamics," and sim-1's
  `env.backend=reduced` selects `Backend::Reduced` — the *PGS* backend, a different thing sharing the
  word "reduced." The implementer correctly ported the diff ABA; the trainer was never moved onto it.

### 🟠 MAJOR — `CudaVecEnv` isn't wired in and can't run the current tasks
- **Not bound.** `csrc/engine_py.cpp` exposes only the CPU `PyVecEnv`; there are **zero**
  `cuda`/`CudaVecEnv`/`ENGINE_CUDA` references in the binding or sim-1 build. The CUDA path is
  engine-internal (built/benchmarked on the A10G); **sim-1 training is unchanged and CPU-only.** The
  reported ~36× is a kernel microbenchmark, not a training result.
- **Missing RL-critical features.** The CPU `PyVecEnv` has `reset_masked` (per-env episode-boundary
  reset) and `set_articulation_state` (RSI) — both **used by sim-1** (tracking/getup rely on RSI).
  `CudaVecEnv` has **neither**; `reset(seed)` resets *all* envs and **ignores the seed**
  (`initStates_.assign(numEnvs_, s0)` ⇒ every env identical, no domain randomization). As-is it cannot
  drive the current tasks or async episode boundaries.

### 🟠 MAJOR — per-step host round-trip caps the payoff
`CudaVecEnv::step()` uploads actions H→D and `packObs()` copies obs D→H **every control step**;
`deviceObservations()/deviceActions()` exist for a zero-copy GPU-policy path but are **unused**. Per the
review's §7.1, a per-step round-trip to a CPU policy roughly doubles step time and serializes the GPU
behind the host — so without the PyTorch-GPU switch + zero-copy wiring, the kernel speed won't translate
to training throughput. (The implementation log flags this correctly as next-step #1.)

### 🟡 MODERATE — the RL wrapper reintroduces CPU/GPU duplication
`actionToTauKernel` (PD servo) and `packObsKernel` (obs) are hand-written **device** kernels that
re-implement the CPU `Environment`'s actuation and `packDefaultObs`. This is exactly the divergence the
core ABA eliminated — reintroduced in the wrapper. Concretely: the PD servo is **not parity-tested**, and
the **ball-joint PD** uses component-wise rotation-vector error (`kp·(target − logSO3(R)) − kd·qd`),
a different formulation from the reduced backend's quaternion-error spherical PD. These should be shared
`ENGINE_HD` free functions (like the ABA), not parallel implementations, or they will drift.

### 🟢 MINOR / notes
- Parity is single-env, raw-`tau` `diffSubstep` only — it does not cover the PD servo, obs packing, or
  the full `CudaVecEnv`. Determinism uses identical init across envs, so "env-to-env bit-identical" is
  nearly tautological (still validates no cross-env contamination — the real point).
- `DiffState<float>` ≈ 760 B/env in local memory (AoS); compute-bound, SoA noted as future — fine.
- `CudaVecEnv` hardcodes `DiffContact::All` and `substeps<=0 → 1` (vs the CPU env's contact selection /
  defaults) — small config-fidelity gaps.

## How the architecture holds up
- **Core (keep as-is):** the one templated ABA + `FlatModel`/`DiffState` POD blobs + clean build gating
  is the durable, correct foundation. Do not change it.
- **RL-env layer (needs a decision):** there are effectively **three** dynamics paths — CPU reduced/PGS
  (what trains today), CPU diff/smoothed (tests only), GPU diff/smoothed (the port) — and the one the
  trainer uses is the one the GPU does not match. The fix is to collapse to **one** RL dynamics shared by
  CPU and GPU (the diff ABA + smoothed contact), which the templating already makes possible.

## The decision this forces
**Unify the RL dynamics on the diff ABA + smoothed contact on both CPU and GPU** (it is already one
templated implementation). Concretely: build a **CPU `DiffVecEnv`** from the same `diffSubstep` + shared
actuator/obs, move sim-1 training onto it, and **retrain** (the banked stand/walk policies are on PGS
contact and won't carry over cleanly). Do NOT port PGS to CUDA (Gauss-Seidel is serial/divergent — the
review rightly punted it). Alternatives (keep two different sims; or port PGS) are worse: the first
breaks CPU↔GPU policy transfer and validation, the second is a large, low-yield effort.

This review feeds two action notes: **engine-side** improvements (shared actuator/obs, a CPU
`DiffVecEnv`, `CudaVecEnv` feature parity, full-env parity tests) in
`2026-07-08-cuda-engine-next-steps.md`; **sim-1-side** integration (bind the env, PyTorch-GPU + zero-copy,
on-GPU glue, the retrain, the Amdahl measurement) in `2026-07-08-cuda-sim1-integration-next-steps.md`.
