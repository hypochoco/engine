# CUDA port — sim-1 integration next steps (DEFERRED)

Dated (2026-07-08). The sim-1-side work to actually turn the engine's GPU physics into a training
speedup. **Deferred** per the owner (engine changes land first; see
`2026-07-08-cuda-engine-next-steps.md`). Lives in the `sim-1` repo (binding + Python trainer), NOT in
this engine — recorded here so the plan travels with the engine that provides the surface. Feeds off the
review (`2026-07-08-cuda-port-code-review.md`).

## The gating decision (do this first — it's a research decision, not code)
**Which dynamics does RL train on?** Today sim-1 trains on the CPU **reduced/PGS** backend
(`env.backend=reduced`). The GPU runs the **diff ABA + smoothed contact**. They are different contact
models (see the review's CRITICAL finding). To get a GPU speedup that produces CPU-deployable policies,
**move RL onto the diff ABA + smoothed contact on both CPU and GPU** and **retrain** — the banked
stand/walk/track policies are on PGS contact and will not transfer faithfully. The engine side makes this
possible (a CPU `DiffVecEnv` + the GPU `CudaVecEnv` running identical templated code). Confirm this
before investing in the binding, because it implies a retrain of the humanoid.

## S0 — Measure the Amdahl ceiling FIRST (still missing)
Profile the current PPO loop and record the **sim % / policy-forward % / learn %** split of wall-time
(e.g. one iteration at `env.num_envs=4096`, `substeps=48`). If sim is only ~50% of wall-time today, even
a perfect kernel caps end-to-end speedup at ~2× — this number decides how much of the below is worth
doing and in what order. It lives here (the trainer), not in the engine. Cheap; do it before S1.

## S1 — Bind the GPU/diff env into `sim1.engine_py`
`csrc/engine_py.cpp` today exposes only `PyVecEnv` (CPU `VecEnv`). Add bindings for the diff envs:
- Expose `physics_env::DiffVecEnv` (CPU diff ABA) and, under `ENGINE_CUDA`, `physics_env::CudaVecEnv`.
- Match the existing `PyVecEnv` surface (`num_envs/act_dim/obs_dim/ndof/nbody`, `actions()`/
  `observations()` zero-copy numpy views, `reset`, `reset_masked`, `set_articulation_state`, `step`,
  `proprio`/`body_*`). The engine-side E3 work (RSI/reset_masked) is a prerequisite for these to exist
  on the CUDA env.
- Build wiring: the sim-1 `csrc/CMakeLists.txt` must pass `-DENGINE_CUDA=ON` (+ the CUDA toolkit hints)
  when building on the NVIDIA box; keep it OFF for the Mac/CPU wheel. `engine_vecenv.py`'s
  `make_vecenv(cfg)` factory picks `cuda` / `diff-cpu` / `reduced` from `cfg`.

## S2 — Keep obs/actions on-device (the real end-to-end win)
The review's precondition (§7.1). `CudaVecEnv` already exposes `deviceObservations()/deviceActions()`
(raw VRAM pointers); the current `step()` still round-trips through host mirrors.
- **PyTorch-GPU policy:** run the actor/critic on the A10G (`torch 2.6.0+cu124` is installed).
- **Zero-copy bridge:** wrap the device obs/action pointers as CUDA tensors (`torch.as_tensor` on the
  device pointer / DLPack / a small binding returning a tensor that aliases the VRAM buffer) so the
  rollout closes on-device with no per-step PCIe copy. Add a device-pointer `step()` path that skips the
  host mirror sync.

## S3 — Vectorize the "glue" on-GPU (avoid the fast-sim-slow-glue trap)
With the sim at ~36×, any per-env CPU/Python reward/termination/`reset()` loop becomes the new
bottleneck. In `sim1/tasks/` these are currently NumPy/torch-CPU per-env:
- Reward terms, termination checks, and `reset_masked` at episode boundaries must operate on the
  GPU-resident obs/state tensors (torch on CUDA), not host loops.
- The NaN divergence guard (`TaskEnv.step`) likewise vectorized on-device.
- RSI / domain-randomization reset hooks: drive the engine `set_articulation_state` (S1) from GPU
  tensors of per-env initial states.

## S4 — Validate + benchmark end-to-end
- **Behavior parity vs CPU:** train a short run on `DiffVecEnv` (CPU) and on `CudaVecEnv` (GPU) with the
  same seed/config; confirm learning curves + `eval` verdict track (tolerance, not bit-exact).
- **Throughput:** re-measure env-steps/s and full-iteration wall-time at 4k/16k/64k envs on the A10G with
  the policy on-GPU and glue vectorized; compare to the S0 baseline to report the *actual* end-to-end
  speedup (expect the review's ~10–30× band, Amdahl-capped, not the ~36× kernel figure).

## S5 — Update the stale sim-1 docs
`sim-1/HANDOFF.md` predates the port and still says "the sim is CPU-only (no CUDA in the engine)". Once
S1–S4 land, update §6 and §8 to describe the GPU env, the dynamics switch/retrain, and the on-device
rollout.

## Dependency order
S0 (measure) → engine E1–E4 (done separately) → S1 (bind) → S2 (on-device obs/actions + GPU policy) →
S3 (on-GPU glue) → S4 (validate/bench) → S5 (docs). S2 and S3 must land together with S1 for a real win;
binding alone (with the host round-trip) gives little.
