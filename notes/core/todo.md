# TODO

Gaps between [goals](goals.md) and [current state](architecture.md), roughly prioritized.
Not commitments — a backlog to reason about.

## Foundational (blocks most other work)

- [~] **ECS core.** DECIDED archetype (2026-07-03); phase 1 landed: `engine::ecs` with
      `Entity` (generational `core::Handle`), `World`, archetype/table storage, and
      `query<Ts...>().each/.chunks`. Ships `engine::Transform` (in `core`). Verified by
      `tst/ecs/unit/entities.cpp`. **Resources + ordered scheduler DONE** (2026-07-03): `World::setResource/
      getResource` + `Schedule` (ordered `void(World&)` systems); `tst/ecs/integration/scheduler.cpp`
      (deterministic gravity→integrate). Next: command buffer + add/remove-component, and
      parallel worlds. **Render-extraction DONE** (2026-07-03):
      `engine::scene` bridge (`RenderMesh`/`RenderMaterial` components + `scene::extract` →
      `RenderView`); `tst/graphics/integration/scene.cpp` + ECS-driven `tst/graphics/visual/grid.cpp`. Plan:
      [2026-07-03-ecs-plan.md](../investigations/2026-07-03-ecs-plan.md).
- [x] **Driver test harness (not a `main`).** DONE (2026-07-03). The engine is a library with
      no application entry point; consuming apps own the loop. A self-registering harness
      (`tst/harness/`, `TST_CASE(module, category, name)`) drives subsystems; tests are organized
      `tst/<module>/<category>/*.cpp` and globbed into three runners — `tests` (unit +
      integration, CTest per module), `benchmarks` (Release), `visuals` (windowed demos). See
      goals.md "Testing / entry points".
- [~] **Break the window/swapchain coupling.** A headless device/context path so ML training
      and offline rendering run without GLFW. The **Metal `Device` has a headless path** (offscreen
      tests render + read back with no window). Remaining: cleanly split `Swapchain` from
      `Renderer` (graphics refactor item 7), and carry the same headless path through the Vulkan
      port. (The parked legacy Vulkan `Graphics` still assumes a window + swapchain end to end.)

## Driving milestone: "a ball rolling down a plane"

See goals.md. Long-term vertical slice: sphere rolling down an inclined plane, plus the
capability targets that make it exercise our goals — headless + deferred rendering, parallel
sims for training, and massive scale (~100k spheres). This is the yardstick for whether core
is "good enough" and defines what the graphics refactor pass must support.

## Next milestone: "a physics humanoid walking on terrain" (RL-ready)

See goals.md + full plan: [2026-07-03-humanoid-rl-milestone-plan.md](../investigations/2026-07-03-humanoid-rl-milestone-plan.md).
Articulated, actuated humanoid on procedural terrain, drivable by keyboard OR an action vector,
lit + steppable headless in parallel batches with batched obs/action tensors. Engine-side
mechanism only — reward/RL-algorithm/cloud/task live in a **downstream sim repo** (see goals.md
"engine ↔ simulation split"). Big open decision: **articulation approach** (maximal-coordinate
joint constraints vs reduced-coordinate Featherstone) — needs its own design doc before Phase B.

### Phase A — Interaction + graphics basics ✅ DONE (2026-07-03)
Backend-agnostic input + ECS-native camera. Split by dependency for extensibility (GLFW today,
Qt/other later) and headless/ML use:
- [x] **`engine::input`** (INTERFACE, GLFW-free): `InputState` (level + edge queries, mouse
      pos/delta incl. `addMouseDelta` for event backends, scroll) + `Key`/`MouseButton`. Adapter
      contract fits polling (GLFW) + event-driven (Qt) + scripted/ML. `tst/input/unit`.
- [x] **`engine::input_glfw`** (adapter): `GlfwInput` polls a window (`void*`) → `InputState`;
      scroll callback; cursor capture. Future `engine::input_qt` sits alongside.
- [x] **`engine::controls`** (ecs + input + core, graphics-free): `FlyController` component +
      `flyControllerSystem` (reads `InputState` + `Time` resources → drives `Transform`).
      `tst/controls/unit/fly_controller.cpp`.
- [x] **Camera as an ECS entity**: render-agnostic `engine::Camera` projection in `core::math`;
      a camera entity = `Transform` + `Camera` (+ optional `FlyController`). `scene::extractViews`
      turns `<Transform, Camera>` → `RenderView`. `engine::Time` frame-dt resource in core.
      `tst/core/unit/camera.cpp`.
- [x] **Lighting**: `render::DirectionalLight` on `RenderView`; renderer packs `GlobalUniforms`
      (viewProj + light) into binding 0; `mesh.slang` does ambient + Lambert (configurable).
- [x] **Background**: `scene::Background` + `scene::SceneLighting` resources + `applyEnvironment`.
      Demo: `tst/graphics/visual/input_demo.cpp` (camera entity + Schedule[input-pump → fly] +
      extractViews + background/light toggles).

### Phase B — Articulated physics (the core; long pole)
- [x] **B0 design doc**: DECIDED (2026-07-03) —
      [2026-07-03-articulation-approach.md](../investigations/2026-07-03-articulation-approach.md).
      **Maximal-coordinate joint constraints first** on the existing impulse solver; add a
      **reduced-coordinate (Featherstone) `PhysicsWorld` backend later** (deferred, but integral
      once training starts — smaller observations → throughput). Keep joint/actuator + obs/action
      API backend-agnostic.
  Full reviewed plan (grounded in the solver + checked vs core goals) in the milestone plan
  §"Phase B — APPROVED PLAN". Joints = persistent velocity constraints in the same solver loop
  as contacts, warm-started across steps; **per-world, allocation-free, no statics** (VecEnv-safe).
- [ ] **B1 Joint constraints core** ← IN PROGRESS: `JointHandle`/`JointType{Ball,Revolute,Fixed}`/
      `JointDef` + `createJoint`/`destroyJoint` in `world.h`; persistent `joints_` store +
      serial warm-started solve (ball = point-to-point 3×3 K; hinge = point + 2 angular; fixed =
      point + 3 angular lock) + Baumgarte. Tests: ball anchors coincide, hinge off-axis ≈ 0,
      fixed keeps relative pose.
- [x] **B2 Joint limits** ✅ (2026-07-03): hinge min/max angle as a one-sided constraint about
      the axis. Test: pendulum settles exactly at the clamp (`hinge_limit_stops`).
- [x] **B3 Actuators + state read** ✅ (2026-07-03): per-Revolute {Torque, PDTarget(kp,kd,target,
      targetVel)} + `maxTorque` (external angular impulse pre-solve). Read `q/qd` single
      (`jointState`) + **bulk SoA span** (`jointStates`); **bulk SoA actuator write**
      (`setJointTargets/Torques`) + single setters. Tests: PD holds a target angle vs gravity,
      torque drives the joint, bulk write/read roundtrip. Ball/Fixed actuation + multi-DOF q/qd
      deferred to B4 (humanoid hips/shoulders).
- [x] **B4 ECS bridge + builder + humanoid preset + self-collision filtering** ✅ (2026-07-03):
      `collisionCategory`/`collisionMask` on `BodyDef` (limbs mask each other out); plain-data
      `ArticulationDef`/`JointSpec` + `buildArticulation` + `makeHumanoid` preset (13 limbs,
      12 joints) in `physics/dynamics/articulation.h`; 3-DOF ball spherical-PD/torque actuation;
      `WorldDef::linear/angularDamping` (drag so free DOFs settle); ECS `Joint`/`JointCommand`
      components + `actuatorFlushSystem` + `spawnArticulation` bridge. Tests: collision_filter,
      ragdoll_settles, ball_actuator_holds_orientation, articulation_ecs. Flat per-joint ball
      q/qd SoA for RL obs deferred (derivable from body poses) until the downstream obs format.
- [x] **B5 Tests + demo** ✅ (2026-07-03): `ragdoll_settles` (passive ragdoll collapses + rests,
      |v|,|ω|→0, no explosion); `pd_stand_holds_pose` (pinned pelvis, PD drives bent knees vs
      gravity + holds neutral elbows); `articulation_determinism` (serial vs parallel bit-identical
      with joints+actuators, err 0); visual `humanoid` (PD-held humanoid on the plane, boxes +
      fly camera). Added `primitives::makeBox` + `WorldDef` damping. Engineering notes (solve
      order, inertia-ratio stiffness, explicit-PD-on-soft-chains) in the articulation note.

**Phase B (articulated physics) COMPLETE** ✅ — maximal-coordinate joints (ball/hinge/fixed) +
limits + actuators (revolute PD/torque + ball spherical PD) + q/qd read + bulk SoA action/obs +
self-collision filtering + articulation builder/humanoid preset + ECS bridge, all deterministic
and green. Next: **Phase D** (env interface) on the flat plane; reduced-coordinate Featherstone
backend + terrain (Phase C) remain deferred.

### Phase C — Terrain ⏸ DEFERRED (2026-07-03) — revisit after B/D
Deferred: a flat `Plane` is enough to get a humanoid balancing + walking; varied terrain is a
refinement, not a prerequisite. Design + rationale + the "GJK/EPA ≠ arbitrary mesh collider"
analysis preserved in
[2026-07-03-terrain-collision-deferred.md](../investigations/2026-07-03-terrain-collision-deferred.md).
Revisit when locomotion needs slopes/stairs/rough ground (e.g. an RL terrain curriculum).
- [ ] (deferred) Heightfield collider (sphere→capsule; AABB-reject broadphase like Plane;
      analytic surface normal to dodge the internal-edge problem).
- [ ] (deferred) Procedural terrain generation in `core::geometry` → heightfield + render mesh.
- [ ] (deferred) Render terrain as a lit static mesh; body-settles-on-terrain test.
- [ ] (later, separate/larger) General concave triangle-mesh collider (convex decomposition, or
      per-triangle + BVH + internal-edge filtering). Convex meshes already work via `ConvexHull`.

### Phase D — RL-ready env interface (on the flat plane) ✅ DONE (2026-07-04)
Reviewed plan: [2026-07-04-phase-d-plan.md](../investigations/2026-07-04-phase-d-plan.md). Headless
`Environment` drives a `PhysicsWorld` **directly (ECS-free)** in the new `engine::physics_env` module.
- [x] **D0 Solver perf micro-opt**: per-body world inverse inertia cached once per substep
      (`computeWorldInvInertia`), read by contacts + joints + actuators + limits. Bit-identical
      (determinism tests still 0.0); win scales with contacts×iterations.
- [x] **D1 `Environment`** (`engine::physics_env`, ECS-free, mirrors `physics_ecs`): `reset(seed)`
      in-place / `setAction` (Torque; actDim=21) / `step` + raw-state accessors (q/qd, root pose+twist,
      contact flags); obs composition downstream + optional default packer (obsDim=53). PhysicsWorld
      gained setBodyState/clearState/refreshState/setJointBallTorque. Tests: dims, bounded rollout,
      deterministic (0.0).
- [x] **D2 `VecEnv`**: N single-threaded worlds on the `ThreadPool` (`parallelFor`, no nesting);
      contiguous SoA `actions()[N×actDim]`/`observations()[N×obsDim]`; reset/resetMasked/step. Test:
      parallel == serial, bit-identical (0.0) at N=24.
- [x] **D3 Determinism + throughput**: single-env + parallel-vs-serial determinism both 0.0;
      benchmark `physics_env.vec_env_throughput` — N=1→1024 ≈ 8.7k→68.7k env-steps/s (13 workers).
      **Engine now has "enough" for the downstream RL repo** (env + batched obs/action).

**Phase D COMPLETE** ✅ — RL-ready mechanism (ECS-free `Environment`/`VecEnv`, in-place reset, raw
batched state + Torque actions, parallel-worlds throughput, determinism) on the flat plane.

### Phase E — Reduced-coordinate Featherstone backend (own track, after D)
Second `PhysicsWorld` (`Backend::Reduced`) behind the same Environment/obs-action API; the largest
single piece of the milestone, slip-able so it doesn't gate RL-readiness. Decision:
[articulation-approach.md](../investigations/2026-07-03-articulation-approach.md).
- [x] **E0** ABA spatial-algebra core (no contacts); validate a free chain/pendulum vs invariants.
      DONE (2026-07-04): `Backend::Reduced` + `src/physics/backends/reduced/featherstone_world.cpp`
      (fixed + floating base, revolute/fixed joints, explicit per-link gravity, 6×6 spatial algebra,
      semi-implicit Euler). Validated in `tst/physics/integration/reduced.cpp`: pendulum period 0.5%,
      double-pendulum energy drift 0.54%, floating free-chain linear/angular momentum 0.1%/0.24%.
      Design + results: [reduced-coordinate-backend.md](../investigations/2026-07-04-reduced-coordinate-backend.md).
- [x] **E1** contact coupling (contact-space inertia + PGS). DONE (2026-07-04): CRBA joint-space
      inertia `H`, generalized contact Jacobians (ancestor revolute cols + floating-base 6 cols),
      sequential-impulse **PGS in generalized coords** (Δq̇ = H⁻¹Jᵀλ; normal λ≥0 + Baumgarte +
      Coulomb friction), dynamic-link colliders (sphere/box/capsule) vs static planes. Validated:
      sphere settles at its radius (no penetration, v→0), box holds on a 17° slope at μ=1 and slides
      at μ=0.05 (friction cone genuine). CRBA cross-checked via the KE identity.
- [x] **E2** humanoid + actuators: **Ball (3-DOF) joints**. DONE (2026-07-04): unified multi-DOF
      rotation-joint model (relRot = restRel·locRot; per-axis S = [axis; −axis×anchorC]) generalizes
      ABA/CRBA/Jacobian/integration from revolute to ball; provably reproduces revolute (E0/E1 stayed
      green). Ball actuation (per-axis torque + spherical PD via quaternion error). `makeHumanoid`
      (14 bodies/13 joints, floating pelvis) runs on `Backend::Reduced`: ragdoll settles on the
      ground (bounded, at rest, no penetration); suspended humanoid holds its neutral pose via PD.
      Tests: ball-pendulum energy 0.04%, ball actuation, humanoid ragdoll, humanoid PD-hold.
- [x] **E3** behind `VecEnv`. DONE (2026-07-04): humanoid `Environment`/`VecEnv` run on
      `Backend::Reduced` by flipping `EnvConfig.backend` only (API unchanged, actDim=21/obsDim=53);
      finite random-torque rollout; single-env determinism + VecEnv parallel==serial bit-identical;
      throughput benchmark reduced vs maximal (~2× slower/step — dense H⁻¹ contact solve dominates;
      reduced needs finer substeps under strong torque). **Phase E COMPLETE.**
- [x] **Phase F — differentiable reduced env (hybrid α-order).** Primary research objective — **engine
      side COMPLETE** (2026-07-04). Review + A/D/hybrid comparison + plan: [2026-07-04-differentiable-reduced.md](../investigations/2026-07-04-differentiable-reduced.md).
      Build (A) analytic differentiable step as the core, (D) zeroth-order over the fast VecEnv as the
      baseline, converge on the α-order hybrid. Forward-mode duals → per-step Jacobian at the env boundary.
  - [x] **F1** differentiable smooth dynamics (no contact) — **COMPLETE** (2026-07-04): `Dual`
        forward-mode AD + Scalar-generic linalg + generalized ABA (`include/engine/physics/diff/{dual,
        linalg,articulated}.h`) for fixed/floating base + revolute/ball/fixed joints, SO(3) exp-map
        integration, quaternion-free. Validated physically (period, energy, momentum) AND for exact
        gradients (revolute/ball/floating all match central FD to 8 digits). Next: per-step
        state/observation Jacobian at the env boundary.
  - [x] **F2** smoothed/compliant contact — **COMPLETE** (2026-07-04). F2a: compliant normal contact
        folded into the generic ABA + `softplus`/`sigmoid` on `Dual`. F2b: regularized Coulomb friction
        (contact-point slip) + radius-lever torque ⇒ sphere rolls without slipping (ω_z=−vx/r); coupled
        hopper (floating base + revolute leg + foot) gradient == FD to 8 digits; soft-contact bias =
        `r−mg/k` (tunable). All gradients through contact validated vs finite differences.
  - [x] **Fd** zeroth-order estimator — **DONE** (2026-07-04): `include/engine/physics/diff/zeroth_order.h`
        (deterministic antithetic Gaussian-smoothing ES gradient). Unbiased on a quadratic (variance ↓ with N);
        agrees with the analytic `Dual` gradient to 0.69% on the smooth pendulum. Both hybrid gradient sources
        now exist. N evals map onto the fast VecEnv downstream.
  - [~] **F3** α-order hybrid blend + per-step Jacobian/VJP at the env boundary + SHAC smoke test.
        **F3a DONE** (2026-07-04): `include/engine/physics/diff/hybrid.h` `alphaOrderGradient` (min-variance
        blend of analytic first-order + zeroth-order). Smooth pendulum ⇒ α=1.0, blend==analytic; stiff
        near-step (β=200) ⇒ α=0.15 + 4.5× lower across-seed variance than pure first-order (Suh et al.).
        **F3b next**: per-step state/observation Jacobian (tangent coords) at the env boundary / `DiffEnvironment`.
  - [x] **F3b Jacobian DONE** (2026-07-04): `include/engine/physics/diff/jacobian.h` `stepJacobian` — per-step
        tangent Jacobian `∂s_{t+1}/∂(s_t,a_t)` via exact forward-mode `Dual<1>` per column; orientation via
        exp/`vee`-log. Full 14×15 floating-base+revolute Jacobian == tangent-space FD to 4e-11 (all blocks).
  - [x] **F3c DONE** (2026-07-04): `from_articulation.h` (`articulationToDiffModel`) + `diff_environment.h`
        (`DiffEnvironment`: reset/setAction/step, `jacobian()`, `rolloutGradient<NA>()`). Real humanoid
        converts (14 links/21 DOF/floating, authored poses reproduced), runs finite/bounded, 54×75
        Jacobian finite, analytic rollout gradient == FD to 1.3e-9. **Phase F engine-side COMPLETE.**
  - [ ] (downstream) SHAC-style short-horizon analytic-policy-gradient smoke test — lives in the RL repo
        (reward/policy/optimizer) consuming `DiffEnvironment` + `alphaOrderGradient`.
  - [x] **Bug-hunting/hardening round DONE** (2026-07-04): unit `diff_invariants.cpp` + integration
        `diff_validation.cpp` + benchmark `diff.cpp` + visual `diff_humanoid.cpp`. Verified converter ==
        production reduced backend (9.2e-8 m), contact gradient == FD (3.7e-10), COM-ballistic, determinism.
        Fixes: **softened contact defaults** (groundK 3e3→2.5e3, C 30→80, β 800→120) + **`DiffEnvironment`
        auto substeps** (contact 48 / free 16) ⇒ passive humanoid drop stable at all substeps. Perf: diff
        forward 0.70× reduced (faster); rolloutGradient ~linear in seeds (NA=21 ≈50× fwd); Jacobian 11 ms.
        Note: [2026-07-04-differentiable-reduced.md](../investigations/2026-07-04-differentiable-reduced.md)
        "Bug-hunting / hardening round".
  - [x] **Contact geometry + stability (F2/F3/F4) DONE** (2026-07-04), plan+results:
        [2026-07-04-differentiable-contact-geometry.md](../investigations/2026-07-04-differentiable-contact-geometry.md).
        (2) multi-point contact mechanism; (3) shape-aware points (capsule caps / box corners, `DiffContact::{None,Feet,All}`);
        (4) semi-implicit contact behind `ContactIntegration` switch. Feature 3's multi-point contact conditioned the
        humanoid so the explicit path is stable to k=8e4 ⇒ restored groundK 2.5e3→1e4 (contact penetration ~0, ragdoll
        rests cleanly), explicit default, semi-implicit available for harder regimes. Full IMEX (M4) not needed.
- [x] **Physics config system (P1–P3) DONE** (2026-07-04), plan+results:
      [2026-07-04-physics-config-system.md](../investigations/2026-07-04-physics-config-system.md).
      Centralized `SimConfig` (`include/engine/physics/config.h`) — un-buried the 10 solver constants
      (`SolverConfig`), `WorldDef`/`EnvConfig` derive from it (no duplication); `SimConfigOverride`+
      `resolve` sparse override layering + `configs.h` named presets; write-only `serialize`/`configHash`/
      `dump`+`configVersion` (`config_io.h`) for run history. Value-type, no global singleton. Next steps ↓.

### Config + downstream / training bring-up (next)
- [ ] **Config: wire `dump()` into the training/VecEnv loop** — log each run's resolved config + hash
      with its results (the actual reproducibility payoff). Gated on the training loop existing.
- [ ] **Config: cross-engine unification** (deferred; user OK'd the two-surface split 2026-07-04) — fold
      the diff engine's contact knobs (`DiffModel`: `groundK/C/beta/mu`, `contactIntegration`) into a shared
      `ContactConfig` in `config.h` so ONE config covers both the RL and differentiable sims. Touches the
      diff engine + layering; do when a single cross-engine tuning/serialization surface is needed.
- [ ] **Config: key/value READER** — parse text → `SimConfig` for launch-time overrides / sweeps without
      recompiling; add when the training launcher consumes config files (the writer already defines the format).
- [ ] **Python bindings** — expose `Environment`/`VecEnv` (+ later `DiffEnvironment`) + `SimConfig` to
      Python for the training stack: batched obs/action tensors, `reset`/`step`, config construction/override,
      and the differentiable surfaces (`rolloutGradient`/per-step Jacobian). Both the PPO (non-diff) and
      SHAC/hybrid (diff) routes to walking consume this. Decide binding tech (pybind11/nanobind) + build wiring.
- [ ] **Adopt a richer humanoid model for mocap training** — replace/augment the hand-built
      `makeHumanoid` preset with a mocap-suitable skeleton (more DOF / realistic proportions / joint limits
      matching a motion-capture retarget target) so imitation / mocap-tracking rewards work. Keep the
      `ArticulationDef` + converter (`articulationToDiffModel`) path so it flows to both backends + the diff engine.
- [ ] **Integrate with the training infrastructure (built elsewhere)** — connect the engine's
      env/config/gradient surfaces to the external trainer (reward/policy/optimizer, experiment configs,
      logging). Define the boundary: Python bindings + config `dump()` + obs/action layout + per-step
      Jacobian/VJP + the α-order hybrid. This is where the SHAC smoke test + PPO path actually run.
- [ ] **Another round of testing** (post-config, pre-training) — unit/integration/benchmark/visual pass
      focused on the config system (override/serialize/hash edge cases), the `Environment`/`VecEnv` boundary,
      and end-to-end batched-rollout determinism/reproducibility before the training stack lands.
- [x] contact-solve perf: **sparse LDLᵀ factorization of `H`** exploiting the DOF-ancestor tree
      (replaces the dense O(ndof³) inverse). DONE (2026-07-04): ~1.5–1.65× env-steps/s for the
      reduced humanoid (N=1024: 16.5k→26.8k); validated vs the dense inverse (≤1.5e-4). When contacts
      are many, the **PGS** dominates instead — future lever: block/Delassus PGS.
- [x] contact-solve perf II: **warm-started PGS + 2×2 block friction (circular cone) + manifold
      reduction** (Delassus skipped — wrong tool when contacts > ndof). DONE (2026-07-04): 20→12
      iters at equal quality; flat-humanoid contact-solve −26% (0.989→0.732 ms), reduced env-steps/s
      +18–22%. End-to-end vs original dense-inverse ≈ 1.9× (N=1024 16.5k→31.3k). Analysis + results:
      [2026-07-04-reduced-contact-pgs.md](../investigations/2026-07-04-reduced-contact-pgs.md).
- [x] reduced-backend coverage hardening (2026-07-04): added `tst/physics/integration/reduced_joints.cpp`
      (fixed/revolute-torque/revolute-PD/off-axis hinge/capsule) + visual gallery
      `tst/physics/visual/reduced_joints.cpp`. Surfaced + fixed two bugs: **joint limits** (were
      ignored → now impulse-based one-sided constraints in the generalized solver) and **WorldDef
      damping on the floating base** (linearDamping ignored → now damps `baseTwist_`). Stress env
      `substeps` 24→48 for the stiffer limited dynamics. See the reduced-coordinate design doc.
- [x] **Ball q/qd multi-DOF readout** (2026-07-04): `JointState` gained `rotation` (rest-relative
      orientation as a rotation vector) + `angularVelocity`; populated by both backends (reduced from
      `locRot`/`qd`, realtime from `refRel`/poses). Default obs packer is now **DOF-complete**
      (revolute→q,qd; ball→rotvec,ω), so humanoid `obsDim` 53→69.
- [x] **PD-target action mode** for the env (2026-07-04): `EnvConfig::actionMode {Torque,PDTarget}`
      + `kp`/`kd`; PDTarget interprets the action as a desired joint position (revolute angle / ball
      orientation-rotvec), tracked by the actuator PD servo. Added `setJointBallTarget` to the
      `PhysicsWorld` interface + both backends. Tests: `reduced_ball_state_readout`,
      `reduced_env_pd_target_tracks` (knee tracks a commanded bend).

## Core (mostly done — geometry/primitives/Handle/Transform/threading landed; image + io remain)

Build `engine::core` to a good state before touching graphics. See
[2026-07-02-core-module-plan.md](../investigations/2026-07-02-core-module-plan.md).

- [x] **geometry/**: `Vertex` (position, **normal**, uv, color), `MeshData`, `Mesh`,
      minimal `Material`, `ModelData`. + `core.h` umbrella. DONE.
- [x] **geometry/primitives**: `makeQuad` / `makePlane` / `makeSphere` generators. DONE
      (feed the milestone + give `tst` real geometry without asset files;
      `tst/core/unit/geometry.cpp`).
- [ ] **image/**: `Image` + `ImageFormat`.
- [ ] **io/**: `readFile`, `loadObj` → `ModelData`, `loadImage` → `Image`. NOTE: centralize
      the stb/tinyobj implementation macros in exactly one TU (they currently live in
      graphics) to avoid duplicate symbols when both link — do this when graphics is
      refactored, or keep loaders' impls out of any target that also links graphics.
- [x] **tst driver**: build meshes via primitives, assert counts, link `engine::core` only.
      DONE (`tst/core/unit/geometry.cpp`).
- [x] Also landed in `core` (not originally listed here): `memory/Handle<Tag>` (shared by rhi +
      ecs), `math/Transform`, and `threading/` (`ThreadPool` + `parallelSort`).

Scaling / ownership (from [2026-07-02-geometry-scaling.md](../investigations/2026-07-02-geometry-scaling.md)):
- [ ] Treat `MeshData`/`ModelData` as **loader output only** — not the runtime store; never
      hold ~100k of them (scattered heap allocs + deep-copy value semantics don't scale).
- [ ] **Central geometry store (pooled vertex/index arenas) + generational `MeshHandle`s** as
      the scalable runtime store; entities reference geometry by handle, not pointer. This is
      the trigger to un-defer `memory/` (`Handle`/`slot_map`) — build the minimal version
      inside this store first, promote to core only if a 2nd consumer needs it.
- [ ] Drive the milestone's 100k spheres via **instancing** (one geometry + 100k transforms),
      with per-frame data in **flat SoA** render/instance arrays (ties to the DrawJob rework).
- [ ] Derive **bounds / position-only data** for physics/culling instead of walking
      interleaved `Vertex` arrays on CPU hot paths.
- [ ] Later/at-scale: **vertex compression** (compact GPU layout, keep core::Vertex as the
      authoring format) and **distinct-geometry streaming/LOD**; decide CPU-retention policy
      (don't keep CPU vertex copies after upload except for collision/raycast).

## Graphics refactor pass (LATER — after core is solid)

**Do NOT refactor graphics incrementally.** Once core is in a good state, refactor the entire
graphics package in one dedicated pass, behind the RHI. Full analysis:
[2026-07-01-graphics-refactor.md](../investigations/2026-07-01-graphics-refactor.md) and
[2026-07-02-metal-backend.md](../investigations/2026-07-02-metal-backend.md).
**Key framing: "add Metal" and "do the refactor" are the same effort** — both require
extracting a backend-agnostic interface (RHI) and putting Vulkan behind it.

- [x] **0. Decide: build-your-own RHI vs. adopt one.** DECIDED (2026-07-02): **build our
      own** — no third-party RHI. Both Vulkan and Metal backends are hand-written.
- [ ] **1. Adopt `engine::core` in graphics**: replace graphics' `Vertex`/`Mesh`/`Material`
      and its OBJ/image loading with core's; delete the now-duplicated code. Each backend
      maps `core::Vertex` to its own vertex format. (Core itself is built in the Core section
      above, before this pass.)
- [x] **2. Define the RHI interface** — DONE (headers, 2026-07-02). Landed under
      `include/engine/graphics/rhi/` (types, resources, pipeline, command_list, device +
      `rhi.h` umbrella) and `include/engine/graphics/render/` (render_view, geometry_store,
      renderer). Interface-only, no backend, compiles clean (`-Wall -Wextra`). Decisions:
      handle-based, compile-time backend, **bindless**, Vulkan dynamic rendering; **Metal
      first**; **Slang** shaders. Design + sequencing:
      [2026-07-02-rhi-interface-plan.md](../investigations/2026-07-02-rhi-interface-plan.md)
      (§13 supersedes the ordering below).
- [ ] **3. Reorganize existing Vulkan code** into `src/graphics/vulkan/` behind the RHI;
      delete the 3 duplicated pipeline builders; fold `graphics_custom.cpp` offscreen helpers
      into the pipeline/pass API. (Design Vulkan side around dynamic rendering to line up
      with Metal.)
- [~] **4. Multi-backend CMake** — mostly DONE (2026-07-02): **Foundation** linked, `external/
      metal-cpp` on the include path, one TU (`metal/metal_backend.cpp`) defines the impl
      macros, **per-backend source selection** (`metal/` vs `vulkan/` + shared `common/`),
      `test`→`tst`, `ENGINE_RHI_METAL/VULKAN` defines. Remaining: enable `OBJCXX` when the
      `.mm` window shim lands (headless Metal is pure C++, so not needed yet).
- [x] **5. Shader toolchain** — DONE (2026-07-03): **Slang** chosen and wired.
      `src/tools/get_slang.sh` fetches `slangc` into gitignored `external/slang/`;
      `src/shaders/CMakeLists.txt` compiles `.slang` → `.metallib` (Apple) / `.spv` (else) and
      exposes `ENGINE_SHADER_DIR`. `rhi::ShaderModule`/`createShader` loads the backend blob.
      First shader: `src/shaders/triangle.slang`.
- [~] **6. Implement the Metal backend** incrementally — offscreen + **windowed** working
      (2026-07-03): `tst/graphics/integration/mesh.cpp` (headless, pixel-verified) and `tst/graphics/visual/grid.cpp`
      (opens a GLFW window, renders a lit `core` sphere via CAMetalLayer swapchain + present).
      Implemented: headless & windowed Device, handle pools, metallib libs, pipeline + vertex
      descriptor + depth-stencil, render-encoder lifecycle, indexed draw, depth, readback,
      CAMetalLayer/drawable present (`metal_window.mm` shim via `glfwGetCocoaWindow`;
      `OBJC`+`OBJCXX` enabled). **MVP camera uniform + instance storage + per-instance materials
      DONE** (see item 8 + the architecture "Build reality" callout). TODO: window resize
      (swapchain + depth recreate; currently fixed-size), staging for Private resources,
      fence-gated deletion, per-frame-in-flight sync, and **bindless textures** (stubs exist).
- [ ] **7. Split `Swapchain` + `Renderer`; headless path** for both backends (ML/offline).
- [~] **8. Rework the render-list / instance path** — instancing + **per-instance materials**
      done (2026-07-03): `RenderView { view/proj, target, RenderItem[], InstanceData[],
      MaterialGPU[] }` consumed by `render::Renderer`; per-instance SoA data + a materials
      storage buffer (each instance's `materialIndex` → `baseColorFactor`); one instanced
      `drawIndexed` per RenderItem. Verified: `mesh_offscreen` (3 differently-colored instanced
      spheres, red center), `visual_window` (NxN color-gradient grid, `ENGINE_GRID=N`).
      Remaining: **bindless texture table** (`baseColorTexture` + `registerBindlessTexture` are
      stubs — defer until textured surfaces are needed; Metal argument-buffer + residency work);
      ECS-driven extraction/culling/sorting once ECS exists; the indirect/GPU-driven path.

Known bugs/smells to fix along the way (all in the **parked legacy Vulkan code** under
`src/graphics/vulkan/` — fix during the Vulkan-behind-RHI port, not present in the Metal path;
details in the refactor investigation):
- [ ] `loadQuad` mis-computes `vertexCount`/`indexCount` (cumulative, not per-mesh counts).
- [ ] `copyInstanceToBuffer` always copies `MAX_ENTITIES`; dirty-range copy is stubbed out.
- [ ] `renderFinishedSemaphores` indexed per-frame not per-image (masked by frames=1).
- [ ] macOS `vkQueueWaitIdle` after every present + `MAX_FRAMES_IN_FLIGHT = 1` → no
      pipelining. Revisit with multiple frames in flight.
- [ ] "paint"/"stamp" naming leftovers from the source app.
- [ ] `GlobalUBO` lights (`// todo: lights`).
- [ ] Deferred deletion queue (readme.md sketch), once resources outlive a frame.

## Compute / ML

- [ ] **Compute pipeline + compute-queue support.** Needed for ML workloads and some
      rendering; only graphics pipelines exist today.
- [ ] **Parallel environment stepping / batching** for training throughput. → concretized by
      Milestone 2 **Phase D** (`Environment`/`VecEnv` over parallel `PhysicsWorld`s + SoA
      obs/action tensors). Foundation exists (parallel worlds, 7.7×).
- [ ] **Determinism review** for reproducible training. → Milestone 2 **Phase D** (end-to-end
      batched-rollout determinism); intra-world stepping is already bit-identical serial↔parallel.

## Physics

Plan (all 8 decisions settled): [2026-07-03-physics-plan.md](../investigations/2026-07-03-physics-plan.md).
Multi-backend behind a **runtime-virtual** `PhysicsWorld`; shared collision/broadphase
substrate; realtime (impulse) + implicit/**differentiable** backends; rotational dynamics.

- [~] **Simulation core.** **Phase 0 + Phase 1 DONE** (2026-07-03): ECS-free `engine::physics`
      core (shapes, exact contacts, rigid-body state + inertia, pure integration kernels incl.
      SO(3) exp/log) + the runtime-virtual **`PhysicsWorld` interface** + a **realtime
      sequential-impulse backend** (restitution/friction/Baumgarte, substeps, rotation) + the
      **`engine::physics_ecs` bridge** (RigidBody + step/sync systems). **Milestone met**:
      `tst/physics/integration/milestone.cpp` (headless — ball rolls without slipping down a 30°
      incline, analytic match) + `tst/physics/visual/rolling.cpp` (windowed, full stack).
      **Phase 2 + collision polish COMPLETE**:
      sweep-and-prune + **uniform-grid** (flat index-sort) broadphases, both verified vs brute
      force. Grid **scales linearly** — at 65,536 bodies grid 19.9 ms/step vs SAP 192 ms vs
      ~14 s for the old O(n²) (~700×); 100k ≈ 30 ms/step single-threaded. **Parallel worlds**
      via `core::ThreadPool`: 7.8× on 12 workers (36.8M body-steps/s). **Colliders**:
      sphere/plane/box/hull/capsule; analytic sphere-box + capsule-sphere/capsule/plane;
      **GJK closest-distance** for capsule-vs-box/hull; GJK/EPA for the normal; **face-clip
      manifolds** (`box_box` + `convex_manifold`) → box/hull **stacking**. **Parallel broadphase
      sort**; **clamped Baumgarte**; **CCD** (swept AABBs + speculative contacts — 120 m/s
      sphere doesn't tunnel); **kinematic bodies move** by scripted velocity (ignore gravity/
      impulses); **restitution works under CCD** (speculative branch targets the rebound velocity
      instead of braking the approach — see
      [2026-07-03-physics-test-findings.md](../investigations/2026-07-03-physics-test-findings.md)).
      Phase 2 + collision polish complete; tests live under `tst/physics/{unit,integration,
      benchmark,visual}/`. Phase 3 (deferred): implicit/differentiable
      backend + parallel-world ML harness.
- [x] **How physics state maps onto ECS components.** DECIDED (2026-07-03): backend owns
      packed state; ECS holds `RigidBody{BodyHandle}` (no pose) + keeps `Transform` separate
      (no replacement/inheritance); a `physics_ecs` bridge syncs world poses → Transform. ML
      path omits Transform entirely. (physics-plan Q2/Q3)
- [ ] **Articulated bodies / joints + actuators** (Milestone 2, Phase B). Joint constraints
      (ball/hinge/limits) + PD/torque actuators + articulation builder + humanoid preset. Big
      open decision: maximal-coord constraints (reuse the impulse solver) vs reduced-coord
      Featherstone/ABA (RL-grade, differentiable-friendly). Needs a design doc first. See
      [2026-07-03-humanoid-rl-milestone-plan.md](../investigations/2026-07-03-humanoid-rl-milestone-plan.md).
- [ ] **Terrain collision** (Milestone 2, Phase C) — ⏸ **DEFERRED** (2026-07-03): flat `Plane`
      suffices for the humanoid for now. Heightfield collider + narrowphase; general concave
      triangle-mesh collider (decomposition or per-triangle+BVH+edge-filtering) is a separate
      larger item — note that **convex** meshes already work via `ConvexHull` (GJK/EPA).
      Design + analysis: [2026-07-03-terrain-collision-deferred.md](../investigations/2026-07-03-terrain-collision-deferred.md).

## Infra / quality

- [~] Multithreaded task system. **`core::ThreadPool` landed** (fixed pool + blocking
      dynamic-work-stealing `parallelFor`, caller participates; `tst/core/unit/thread_pool.cpp`).
      Uses: **parallel worlds** 7.7× on 12 workers, and **intra-world** parallel integration +
      narrowphase + **graph-colored contact solver** (deterministic, bit-identical to serial;
      1.66× on a dense 32k pile) + **parallel broadphase sort** (`core::parallelSort`, landed).
      Still TODO: a task **dependency graph** (readme sketch), fewer per-color barriers.
- [x] Test setup. DONE (2026-07-03): self-registering harness + `tests`/`benchmarks`/`visuals`
      runners + CTest per module (see the Foundational "Driver test harness" item).

## Open design questions (revisit "at some point")

Parked decisions we agreed to defer. Answer before the work they gate.

- [x] **ECS approach: archetype vs sparse-set.** DECIDED (2026-07-03): **archetype** — best
      multi-component iteration/cache, maps onto batched instancing + the 100k case; structural
      changes (rare in a sim) are its weak spot but acceptable. (ecs-plan §3)
- [ ] **ML training model: many envs in-process vs separate processes.** Drives the
      headless context design and how far determinism must reach. (Lean: in-process vectorized
      via `ThreadPool` first — Milestone 2 Phase D; multi-process/distributed is downstream/cloud.)
- [~] **Articulation approach (Milestone 2): maximal-coordinate joint constraints vs
      reduced-coordinate (Featherstone/ABA).** DECIDED (2026-07-03):
      [2026-07-03-articulation-approach.md](../investigations/2026-07-03-articulation-approach.md).
      **Constraints first** (Phase B — fast, reuses our solver, demonstrates the humanoid);
      **reduced-coordinate deferred but integral once we train** — it minimizes observation size
      (joint `q/qd` vs redundant body poses) → smaller policy inputs → higher throughput (a key
      training bottleneck), plus drift-free stability + cleaner gradients. Added later as a second
      `PhysicsWorld` backend; keep the joint/actuator + obs/action API backend-agnostic.
- [ ] **Terrain representation: heightfield vs general triangle-mesh collider.** Heightfield
      recommended (matches locomotion RL); tri-mesh possibly later.
- [ ] **Offline renderer basis:** is `graphics_custom.cpp`'s offscreen helper set intended
      to grow into the offline/high-quality renderer?
- [x] **Build-your-own RHI vs. adopt.** DECIDED (2026-07-02): build our own; no third-party
      RHI. (metal-backend §3)
- [x] **Shader language/toolchain**: DECIDED (2026-07-03) — **Slang** (one source → SPIR-V +
      metallib), wired via `slangc`. (metal-backend §5)
- [ ] **RHI dispatch**: runtime-virtual (simpler) vs. compile-time (zero overhead).
- [ ] **Metal API level**: classic `MTL` (widest support) vs. `MTL4` (newer, Vulkan-like;
      present in vendored metal-cpp).
- [ ] **Future D3D12 / Windows?** If likely, argues harder for adopting an RHI now.
