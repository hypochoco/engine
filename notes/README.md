# Engine Notes

Working notes for the engine project.

## Structure

- **`core/`** — living documents, kept up to date over time:
  - [`goals.md`](core/goals.md) — what the engine is for and the constraints that follow
  - [`architecture.md`](core/architecture.md) — current structure of the codebase, as-is
  - [`todo.md`](core/todo.md) — prioritized work, gaps between goals and current state

- **`investigations/`** — dated, point-in-time deep-dives. Named `YYYY-MM-DD-topic.md`.
  These are snapshots: they are *not* updated after the fact. When conclusions change,
  fold them back into the `core/` docs and write a new investigation.
  - [`2026-07-01-codebase-overview.md`](investigations/2026-07-01-codebase-overview.md)
  - [`2026-07-01-graphics-refactor.md`](investigations/2026-07-01-graphics-refactor.md)
  - [`2026-07-02-metal-backend.md`](investigations/2026-07-02-metal-backend.md)
  - [`2026-07-02-core-module-plan.md`](investigations/2026-07-02-core-module-plan.md)
  - [`2026-07-02-geometry-scaling.md`](investigations/2026-07-02-geometry-scaling.md)
  - [`2026-07-02-rhi-interface-plan.md`](investigations/2026-07-02-rhi-interface-plan.md)
  - [`2026-07-03-ecs-plan.md`](investigations/2026-07-03-ecs-plan.md)
  - [`2026-07-03-physics-plan.md`](investigations/2026-07-03-physics-plan.md)

## Conventions

- `core/` docs are the source of truth for "how things are now" and "where we're going."
- Investigations capture reasoning at a moment in time — keep them even when superseded.
- When an investigation drives a decision, update the relevant `core/` doc and link back.
