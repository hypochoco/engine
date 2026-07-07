# CUDA port review — what it would take to run the physics sim on the GPU

Dated review (2026-07-06). **This is a review + recommendation, not an implementation.** It assesses
what a GPU (CUDA) port of the physics simulation would require, which parts of the codebase port
cleanly vs fight it, the concrete blockers, and a recommended target + phasing. Grounds against the
current backends (`src/physics/backends/reduced/featherstone_world.cpp`,
`src/physics/backends/realtime/sequential_impulse_world.cpp`), the differentiable core
(`include/engine/physics/diff/`), and the env boundary (`src/physics_env/`).

Ties into: goals.md ("ML training — headless, high-throughput, many parallel environments,
deterministic"; "Compute, not just graphics"), the parallel-worlds throughput thesis
(`2026-07-04-phase-d-plan.md`, `2026-07-03-humanoid-rl-milestone-plan.md`), and the differentiable
track (`2026-07-04-differentiable-reduced.md`).

## 0. Target hardware (the framing decision)

**The dev machine is a Mac (Metal backend); the GPU is on a separate machine with an NVIDIA card.**
This is the deployment split the engine already anticipates — the `ENGINE_TRAINING_ONLY` CMake path
exists precisely to build "a headless Linux box with no GPU/display SDK" (top `CMakeLists.txt`).

Consequences that shape the whole port:
- **CUDA does not run on macOS.** The CUDA physics backend will build and run **only** on the
  NVIDIA/Linux machine, never on the Mac. Development is cross-platform by necessity: the Mac keeps
  building the CPU backends (`Realtime`/`Reduced`) + Metal; CUDA is gated behind `if(NOT APPLE)` and
  a `-DENGINE_CUDA=ON` option.
- This is a natural extension of the existing backend enum (`config.h` `Backend{ Realtime, Reduced }`
  → add `Cuda`), not a new architecture.
- The Mac can't run the CUDA tests; parity/regression tests for the GPU path run on the NVIDIA box
  (CI or manual). Plan for a remote build/test loop from day one.

**Target machine (AWS g5.4xlarge-class, captured 2026-07-06):**
- **GPU: NVIDIA A10G** — Ampere GA102, compute capability **`sm_86`**, 72 SMs / 9216 FP32 cores,
  **24 GB** GDDR6 (~600 GB/s). FP32 peak ≈ **31 TFLOP/s**; **FP64 is 1/32 rate ≈ 1 TFLOP/s** ⇒
  `double` is for offline gradient-checks only, never the hot loop. Tensor cores (TF32/FP16) exist —
  relevant to the **policy network**, not the physics kernel. Driver 595.71.05, **CUDA 13.2**.
  Shared with a desktop session (Xorg/gnome/DCV using ~1.7 GB) — it's an interactive cloud desktop,
  not a dedicated trainer, so expect occasional contention.
- **CPU: AMD EPYC 7R32** — **8 physical cores / 16 threads**, 32 MB L3. Modest core count ⇒ the CPU
  baseline on *this* box is below the ~31k env-steps/s figure (measured on a ~12–13-worker machine),
  and the CPU cannot feed a GPU-hungry pipeline if anything stays host-side.
- **Software:** PyTorch **CPU** today; the plan is to switch to **PyTorch GPU** once the sim is
  CUDA-friendly (this is the precondition that makes the port pay off — see the perf note §6).

## 1. The good news: the architecture is already GPU-shaped

The usual hard part of a GPU physics port — rearchitecting for batch parallelism + determinism — is
largely done, for CPU reasons that transfer directly:

- **Throughput comes from parallel *worlds*, not intra-world threads** (`phase-d-plan.md`). That is
  exactly the GPU model: one thread (or warp) per environment, thousands resident at once — the
  Brax / Isaac Gym / MuJoCo-MJX design.
- **The `VecEnv` boundary is already plain-data SoA.** `vec_env.cpp` holds flat `actions_`/`obs_`
  `float` arrays indexed `i*dim`, filled by `Environment::packDefaultObs`. A CUDA backend fills the
  *same* buffers — the RL-facing contract doesn't change. Ideally the buffers live in device memory
  and only cross to host when the policy needs them (or never, if the policy is also on-GPU).
- **No hidden global state; determinism is a hard gate** (bit-identical serial vs parallel across the
  whole physics stack). Per-env independence is what lets each env map to one CUDA thread with no
  cross-thread hazards.
- **The differentiable core is already STL/glm-free in its math and scalar-generic.** `diff/linalg.h`
  defines fixed-size `V3/M3/V6/M6` (`S m[3][3]`), templated on scalar `S`, quaternion-free (SO(3)
  exp map via `rodrigues`/`expSO3`). That templating + fixed layout is ~80% of what device code needs.

So the question isn't "can this run on a GPU" — the design answers yes. It's "which backend do we
port, and what must change in the step body to make it device-executable."

## 2. What ports well, what fights you

**Best candidate — the reduced/differentiable ABA** (`diff/articulated.h`; algorithm mirrors
`featherstone_world.cpp`).
- Fixed topology (14-body / 21-DOF humanoid; also the 28-DOF AMP rig — both are `ArticulationDef`
  data), small dense per-joint math (3×3, 6×6 in `linalg.h::invertSmall`/`solveM6`). Identical control
  flow across all envs, tiny state, no data-dependent sizing once topology is baked → ideal SIMT.
- The differentiable path uses **smoothed compliant contact** (`groundContactWorld`: softplus/sigmoid,
  no active set, no PGS iteration) — branch-free, fixed work per contact point. Far more GPU-friendly
  than the hard solver. MJX/Brax run smoothed/compliant contact on GPU for exactly this reason.
- Already scalar-generic, so a `__device__` instantiation for `float` (and `double` for grad-checks)
  is closer to a compile than a rewrite.

**Hardest candidate — the maximal-coordinate `sequential_impulse_world.cpp`.**
- Sequential impulse is **Gauss-Seidel**: each constraint reads the velocity the previous one wrote —
  inherently serial *within a world*. One-world-per-thread still works, but parallelizing a single
  large scene needs graph-coloring / Jacobi reformulation (`terrain-collision-deferred.md` anticipates
  a "graph-colored parallel solve").
- Warm-start caches (`impulseCache_`), variable-size contact manifolds, and runtime body/joint
  creation are all awkward on device.

**Divergence-heavy — collision** (GJK/EPA, SAP, uniform grid). GJK/EPA are iterative with
data-dependent loop counts (`gjk.cpp`, `epa.cpp`) → warp divergence; SAP needs a sort. Doable (Isaac
does it) but it's the messy part. For a fixed humanoid-on-plane/terrain RL task the smoothed analytic
contact sidesteps most of it.

## 3. Concrete blockers (what nvcc rejects or what silently kills perf)

1. **Virtual dispatch.** `PhysicsWorld` is abstract with `virtual step()` (`world.h`). Vtables don't
   belong in hot device code — the batched kernel calls a concrete `__device__` step directly; the
   vtable stays host-side (orchestration only).
2. **`std::vector`/`std::unique_ptr` in the hot loop.** `diffForwardDynamics` allocates ~a dozen
   `std::vector`s per call (`Xup, IA, v, c, pA, a, Scol, U, Dinv, ...`). On device these become
   fixed-size stack arrays sized by compile-time `MAX_LINKS`/`MAX_DOF`. **This is the single biggest
   mechanical change to the diff code.**
3. **Dynamic topology.** `createBody`/`createJoint` grow vectors at runtime. GPU wants the model
   **baked once** into flat SoA constant arrays (`parent[]`, `dof[]`, `qIndex[]`, `axes[]`,
   `inertia[]`, ...) and uploaded; per-env mutable state is the only per-step buffer. Reuse
   `from_articulation.h` to produce the flat model.
4. **Precision policy.** Production is `Real = float` (`types.h`); the diff engine is hard-coded
   `double` ("gradient/FD agreement needs more than float"). On consumer NVIDIA cards `double` is
   1/32–1/64 throughput. Plan: **float for the forward RL sim**, `double` **only** for offline
   grad-check tests. The `Real` typedef localization was designed for exactly this swap.
5. **glm in the production backends.** `featherstone_world.cpp` uses glm `Vec3/Quat/Mat3` throughout.
   glm *can* compile for CUDA (`GLM_FORCE_CUDA`), but the diff module's hand-rolled `linalg.h` is the
   cleaner device substrate — another reason it's the better port target.
6. **Determinism across the batch.** Per-env independence keeps each env bit-reproducible (the gate we
   hold). But avoid cross-env atomics/reductions in the step, and note that float results **will
   differ CPU↔GPU** (FMA/rounding order) — determinism tests need a "same-device" qualifier, and any
   CPU↔GPU parity test must be tolerance-based, not bit-exact.
7. **Build system.** `enable_language(CUDA)`, a separate-compilation target, `-DENGINE_CUDA=ON` gated
   `if(NOT APPLE)`. Slots under a new `Backend::Cuda` in `config.h`, out of the Apple build.

## 4. Recommended shape of the port

1. **Confirm the target GPU/specs** (§0 TODO) before writing any `.cu`.
2. **Port the reduced + smoothed-contact ABA, batched one-env-per-thread** — not the maximal solver,
   not intra-world parallelism. Matches the parallel-worlds thesis, the RL use case, and MJX/Brax
   precedent.
3. **Keep the `VecEnv` boundary identical.** A `CudaVecEnv` fills the same flat `obs_`/`actions_`
   buffers; keep them in device memory, cross to host only when the policy needs them.
4. **Bake the model to flat SoA once**, upload, then the per-step kernel touches only per-env state.
5. **Reuse the scalar-generic templates.** With fixed-size arrays the `diff/` math can plausibly
   compile `__device__` for `float` (forward) and `Dual<N>` (on-device gradients) — a real payoff for
   the differentiable-sim research track, and consistent with "build everything ourselves"
   (no Warp/Brax adoption).
6. **Leave collision as smoothed-analytic for the RL task**; defer GJK/EPA/SAP on GPU until arbitrary
   convex-convex on device is actually needed.

## 5. Effort & risk

- **Reduced + smoothed-contact batched forward kernel** — the tractable core. Weeks, not days: mostly
  (a) fixed-size-ifying `diffForwardDynamics`, (b) the batched kernel + buffer management, (c) a CUDA
  CMake target + CPU↔GPU parity tests. **Medium** risk.
- **On-GPU gradients (`Dual` on device)** — natural extension once the forward kernel exists; a real
  win for the diff track. **Medium** risk (register pressure from `Dual<N>` partials).
- **Maximal solver / general collision on GPU** — high effort, high divergence risk, largely
  unnecessary for humanoid-RL. **Punt.**
- **Biggest non-technical risk:** the cross-platform loop (build/test only on the remote NVIDIA box).
  Stand this up early.

## 6. Open items / next

- **Performance analysis (A10G):** see §7 below.

## 7. Performance (A10G) — estimates, pending real measurement

> **⚠️ These are order-of-magnitude estimates from FLOP/bandwidth/Amdahl reasoning, NOT measurements.**
> Nothing here was run on the A10G (the review was done from the Mac). Firm these up with the two
> measurements in §7.5 before trusting any number for planning.

### 7.1 The precondition: everything on-GPU, obs/actions never leave VRAM
Humanoid obs ≈ 69 floats (`defaultObsDim`: `13 + 2·ndof + nbodies`). At 32k envs that's ≈ 8.8 MB
obs + similar actions; over PCIe gen4 (~25 GB/s realistic) ≈ 0.35 ms **each way**, vs an A10G step of
~0.5–2 ms for 32k envs. So a **per-step round-trip to a CPU policy roughly doubles step time and
serializes the GPU behind the CPU** — little or negative gain. The **PyTorch-CPU → PyTorch-GPU switch
is therefore the precondition**, not a nice-to-have: with the policy on the A10G the obs/action
buffers stay in VRAM, the transfer term → 0, and the rollout loop closes on-device. The physics port
and the PyTorch-GPU switch must land together or neither helps.

### 7.2 Raw physics-sim speedup: ~20–40×
- Per env per control step: ~48 substeps (the contact default) × an ABA eval of ~tens of kFLOP for a
  14-link humanoid → order **1–3 MFLOP/env/step**.
- The A10G won't approach its 31 TFLOP FP32 peak — ABA is register-heavy, branchy, low
  arithmetic-intensity; realistic sustained ~5–15% of peak (~2–5 TFLOP/s effective). ⇒ ~1M–2.5M
  env-steps/s theoretical, haircut for occupancy/divergence → **a good kernel likely lands
  ~300k–800k env-steps/s**.
- Against a realistic **~15–25k env-steps/s on *this* 8-core box** (below the 31k figure measured on a
  bigger machine, and worse in practice while PyTorch-CPU steals cores), that's **~20–40× on the
  physics step alone**.
- **Sizing:** want **~16k–65k envs** to saturate 72 SMs; per-env state is a few KB, so even 65k envs
  is a few hundred MB of 24 GB ⇒ **compute-bound, not memory-bound** (the good failure mode).

### 7.3 End-to-end training speedup: ~10–30× (Amdahl bites)
Training wall-time ≈ `rollout (sim + policy-fwd) + learn (PPO/SAC minibatch epochs)`. Two erosions of
the sim number:
1. **Once the sim is ~30× faster, the learning phase stops being free.** PPO's minibatch backprop,
   negligible next to CPU sim today, becomes a real share on GPU (it also moves to the A10G and speeds
   up via tensor cores — a shift in balance, not a new wall). If learning ends up ~40% of the loop,
   Amdahl caps end-to-end speedup regardless of sim speed.
2. **The "fast sim, slow glue" trap.** Reward, termination, and `reset()` (`environment.cpp`
   `applyInitialState`/`updateContactFlags` + downstream reward/curriculum in the `sim1` repo) must
   **also** be vectorized on-GPU. Any per-env CPU/Python loop there becomes the new bottleneck — the
   most common reason GPU-sim ports underdeliver.

Net (everything on-GPU + glue vectorized): **~10–30× faster end-to-end training** on this A10G vs
CPU-everything on this box — high end early (rollout-dominated), lower as learning's share grows. The
Isaac Gym/Brax story at single-mid-GPU scale: ~1–1.5 orders of magnitude, not the "1000× vs a CPU
cluster" headline.

### 7.4 Highest-leverage lever is algorithmic, not the GPU
The **48 substeps/control-step** multiplies every backend's cost. Per `differentiable-reduced.md`
(F4), soft-but-stable explicit contact forces those 48 substeps; a **semi-implicit/implicit contact
solve decouples stiffness from timestep**. Dropping to 8–16 substeps is a **3–6× throughput win on
top of the port** (and helps the CPU path too). Treat implicit contact as a co-priority with the
kernel — it may be worth more than squeezing occupancy.

### 7.5 Replace guesses with measurements (do these before committing)
1. **Today's baseline:** profile the downstream PPO loop (`sim1`) and record the **sim % /
   policy-forward % / learn %** split. This gives the Amdahl ceiling *before* any CUDA is written — if
   sim is only ~50% of wall-time today, the port caps at ~2× no matter how fast the kernel is.
2. **After a prototype kernel:** micro-benchmark `envs × step()` on the A10G at 4k/16k/64k envs to
   find the saturation point and real env-steps/s.

## Decision

If the NVIDIA machine is the training target: **add a `Backend::Cuda` reduced + smoothed-contact ABA,
batched one-env-per-thread, behind the existing `VecEnv` boundary, gated `if(NOT APPLE)`.** Reuse the
scalar-generic `diff/` math (fixed-size-ified) so forward `float` and on-device `Dual` gradients share
one implementation. Defer the maximal solver and general GPU collision. Not started — pending the
hardware specs and the performance pass (§6).
