# Engine Notes

Working notes for the engine project.

## Structure

- **`core/`** — living documents, kept up to date over time:
  - [`goals.md`](core/goals.md) — what the engine is for and the constraints that follow
  - [`architecture.md`](core/architecture.md) — current structure of the codebase, as-is
  - [`todo.md`](core/todo.md) — prioritized work, gaps between goals and current state

- **`investigations/`** — dated, point-in-time deep-dives. Named `YYYY-MM-DD-topic.md`.
  These are snapshots: they are *not* updated after the fact. When conclusions change,
  fold them back into the `core/` docs and write a new investigation. Grouped by domain:
  - **`core/`** — the backend-agnostic foundation (module layout, ECS, geometry) shared by every track
  - **`physics/`** — the physics engine + RL/differentiable simulation track
  - **`realtime-rendering/`** — the realtime render path (incl. the RHI + Metal backend it stands on)
  - **`path-tracing/`** — the (planned) offline path-tracer track

### `investigations/core/`

- [`2026-07-01-codebase-overview.md`](investigations/core/2026-07-01-codebase-overview.md) — first pass over the WIP engine's structure and standing
- [`2026-07-02-core-module-plan.md`](investigations/core/2026-07-02-core-module-plan.md) — `engine::core`, the backend-agnostic foundation module
- [`2026-07-02-geometry-scaling.md`](investigations/core/2026-07-02-geometry-scaling.md) — how core geometry types scale toward massive-scale (cache/ownership)
- [`2026-07-03-ecs-plan.md`](investigations/core/2026-07-03-ecs-plan.md) — `engine::ecs` design (data-oriented, determinism, parallel envs)

### `investigations/physics/`

- [`2026-07-03-physics-plan.md`](investigations/physics/2026-07-03-physics-plan.md) — the physics engine plan
- [`2026-07-03-physics-baseline.md`](investigations/physics/2026-07-03-physics-baseline.md) — performance baseline (pre-scaling)
- [`2026-07-03-physics-test-findings.md`](investigations/physics/2026-07-03-physics-test-findings.md) — test-coverage findings
- [`2026-07-03-articulation-approach.md`](investigations/physics/2026-07-03-articulation-approach.md) — maximal-coordinate constraints vs reduced-coordinate (Featherstone)
- [`2026-07-03-terrain-collision-deferred.md`](investigations/physics/2026-07-03-terrain-collision-deferred.md) — terrain/heightfield collision, deferred (revisit)
- [`2026-07-03-humanoid-rl-milestone-plan.md`](investigations/physics/2026-07-03-humanoid-rl-milestone-plan.md) — Milestone 2: an RL-ready humanoid walking on terrain
- [`2026-07-04-phase-d-plan.md`](investigations/physics/2026-07-04-phase-d-plan.md) — Phase D RL-ready env interface; Phase E Featherstone backend
- [`2026-07-04-reduced-coordinate-backend.md`](investigations/physics/2026-07-04-reduced-coordinate-backend.md) — Phase E reduced-coordinate Featherstone `PhysicsWorld` backend
- [`2026-07-04-reduced-contact-pgs.md`](investigations/physics/2026-07-04-reduced-contact-pgs.md) — reduced-backend contact PGS analysis + optimization
- [`2026-07-04-differentiable-reduced.md`](investigations/physics/2026-07-04-differentiable-reduced.md) — differentiable reduced-coordinate environment design
- [`2026-07-04-differentiable-contact-geometry.md`](investigations/physics/2026-07-04-differentiable-contact-geometry.md) — differentiable contact geometry + stability
- [`2026-07-04-diff-semiimplicit-testing.md`](investigations/physics/2026-07-04-diff-semiimplicit-testing.md) — differentiable semi-implicit contact testing round
- [`2026-07-04-humanoid-rig-adoption.md`](investigations/physics/2026-07-04-humanoid-rig-adoption.md) — rig-agnostic model support + AMP humanoid plan
- [`2026-07-04-physics-config-system.md`](investigations/physics/2026-07-04-physics-config-system.md) — centralized physics config system (P1–P3)

### `investigations/realtime-rendering/`

- [`2026-07-01-graphics-refactor.md`](investigations/realtime-rendering/2026-07-01-graphics-refactor.md) — breaking up the `Graphics` god-object
- [`2026-07-02-metal-backend.md`](investigations/realtime-rendering/2026-07-02-metal-backend.md) — Metal backend / multi-backend review
- [`2026-07-02-rhi-interface-plan.md`](investigations/realtime-rendering/2026-07-02-rhi-interface-plan.md) — the graphics interface (RHI) plan
- [`2026-07-04-render-framework-plan.md`](investigations/realtime-rendering/2026-07-04-render-framework-plan.md) — clustered forward+ render graph (framework design)
- [`2026-07-05-sky-atmosphere.md`](investigations/realtime-rendering/2026-07-05-sky-atmosphere.md) — procedural sun-coupled sky pass (RF6 sky)
- [`2026-07-05-atmosphere-aerial-perspective.md`](investigations/realtime-rendering/2026-07-05-atmosphere-aerial-perspective.md) — aerial-perspective + height fog (RF6 atmosphere)
- [`2026-07-05-antialiasing-msaa-fxaa.md`](investigations/realtime-rendering/2026-07-05-antialiasing-msaa-fxaa.md) — MSAA + optional FXAA anti-aliasing (RF6 AA)
- [`2026-07-05-graphics-config-system.md`](investigations/realtime-rendering/2026-07-05-graphics-config-system.md) — centralized GraphicsConfig + RenderResources (G1–G3)

### `investigations/path-tracing/`

- [`2026-07-06-renderer-head-swap-readiness.md`](investigations/path-tracing/2026-07-06-renderer-head-swap-readiness.md) — module dependency review: swapping the graphics head (realtime ↔ path tracer)

## Conventions

- `core/` docs are the source of truth for "how things are now" and "where we're going."
- Investigations capture reasoning at a moment in time — keep them even when superseded.
- New investigations go in the matching domain subdir, named `YYYY-MM-DD-topic.md`.
- When an investigation drives a decision, update the relevant `core/` doc and link back.
