# Humanoid rig adoption — rig-agnostic model support + AMP humanoid plan

**Date:** 2026-07-04
**Context:** we want a mocap-friendly humanoid for imitation/tracking training. Research corpus in
`~/Projects/research/notes/` (`2026-07-04-humanoid-model-adoption.md`, `-skeleton-standards.md`,
`core/recommended-pipeline.md`) + reference code in `~/Projects/research/humanoid-motion/` (ASE/AMP
MJCF + poselib retargeting + mocap data). This note is the engine-side build plan.

## Decision: rig-agnostic engine, support BOTH rigs (defer the per-run choice)
A rig is **data** (`physics::ArticulationDef`), not code — the converter (`from_articulation.h`),
`DiffEnvironment`, and both physics backends operate on any `ArticulationDef`. So we keep multiple
rig factories side by side and select per experiment (via `SimConfig`):
- `makeHumanoid()` — existing **14-body / 21-DOF** rig (bootstrap default; already validated + wired).
- `makeAMPHumanoid()` — **15-body / 28-DOF** DeepMimic/AMP rig (authored later, for mocap tracking).
- `makeSmplHumanoid()` — only if we commit to the AMASS/SMPL data ecosystem.

**Shared / built once (rig-independent):** the engine, the diff converter, obs/action layout (already
DOF-derived), and the per-joint physics features below. **Inherently per-rig:** trained policy weights
(different obs/act dims ⇒ can't share across topologies) and the retarget config. So the infra supports
either rig; a given training run commits to one.

## The reference rig (from `ASE/.../mjcf/amp_humanoid.xml`)
15 bodies, floating pelvis, **28 actuated DOF**: abdomen(3) + neck(3) + shoulder(3)×2 + elbow(1)×2 +
hip(3)×2 + knee(1)×2 + ankle(3)×2; hands are rigid tip bodies (no DOF). Feet are boxes (matches our
box-corner contact). Each joint carries gear (→ maxTorque), passive stiffness, damping, armature, limits.
**Gap vs our `makeHumanoid` (21 DOF):** +3-DOF neck (we have it fixed) and ankles 1-DOF→3-DOF (+4) = +7.
Every joint TYPE AMP needs (ball/revolute/fixed/floating/limits/actuators) we already support ⇒ authoring
`makeAMPHumanoid` is a data task, once the per-joint physics features exist.

## Rig-agnostic physics features to implement NOW (this note's scope)
These make the engine able to faithfully express an MJCF/URDF rig; all default to no-ops (existing models
unchanged). Implemented in the differentiable engine (`DiffModel`/`DiffLink` + ABA); reduced-backend
parity is a follow-up.
1. **Per-joint viscous damping** — promote `jointDamping` to `DiffLink::jointDamping` (`<0` ⇒ inherit the
   `DiffModel` global). MJCF damping is per-joint (0.1 … 100).
2. **Per-joint passive stiffness** — `DiffLink::jointStiffness`: a spring `τ = −k·q` pulling the joint
   toward its rest pose (q = joint rotation vector · axis, via `logSO3`). MJCF joints have springs.
3. **Armature (rotor/reflected inertia)** — `DiffLink::armature`: added to the joint-space inertia
   diagonal `D` in ABA pass 2. Stabilizes stiff joints; MJCF sets it per joint.

## Deferred (with the rig, not now)
- Authoring `makeAMPHumanoid()` (transcribe the MJCF: bodies, **mass = geom volume × density**, joint
  axes/limits, gears→maxTorque, per-joint stiffness/damping/armature; **Z-up→Y-up** conversion).
- Re-validating the converter + Jacobian at the new topology; obs/action dims grow (21→28).
- The retargeting pipeline (poselib is Python: `joint_mapping` + T-poses + scale/rotation/root-height →
  reference clips on our rig). Run offline; reuse AMP's pre-retargeted walk/run/jog + Reallusion combat
  clips directly IF `makeAMPHumanoid` matches AMP topology exactly.
- Reduced-backend parity for the new per-joint props (diff engine is the training/tracking path first).

## Reference assets inventory (`~/Projects/research/humanoid-motion/ASE/ase/`)
- `data/assets/mjcf/amp_humanoid.xml` (+ `_sword_shield.xml`) — the rig spec.
- `poselib/` — retargeting lib + `data/configs/retarget_{cmu,sfu}_to_amp.json` + T-poses + CMU/SFU FBX.
- `data/motions/` — `amp_humanoid_{walk,run,jog}.npy` (on the AMP rig) + ~87 Reallusion sword/shield clips.
- Motion format: `SkeletonMotion` `.npy` (per-frame local joint rotations + root translation).

## Build order
1. **Now:** per-joint damping + passive stiffness + armature (diff engine) + tests. ← this note.
2. Author `makeAMPHumanoid()` (data) + converter re-validation, when starting mocap tracking.
3. Offline retargeting (poselib) → reference clips on the chosen rig.
4. Reduced-backend parity for the per-joint props (if the RL sim needs them).
