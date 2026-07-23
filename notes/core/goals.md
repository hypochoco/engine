# Goals

## What the engine is for

A single engine serving three workloads on shared foundations:

1. **Realtime simulation / games** — interactive framerates, windowed presentation.
2. **Machine learning training** — likely headless, high-throughput, many parallel
   environments, deterministic where possible.
3. **Rendering** — offscreen/offline image generation (higher quality, not necessarily
   realtime).

## Architectural commitments

- **Build everything ourselves.** The purpose of this project is to develop these systems
  in-house — no adopting third-party engines/RHIs (e.g. Dawn, bgfx, SDL_gpu). Small focused
  libraries for narrow concerns (glm, stb, tinyobjloader, glfw, metal-cpp bindings) are
  fine, but the engine's own abstractions (RHI, ECS, renderer, physics) are hand-written.
- **Engine, not application.** This is a library other projects consume to build their own
  games/sims/tools. There is intentionally **no `main`** — the engine exposes APIs. See
  the testing note below for how the drivers are exercised.
- **Entity Component System (ECS)** as the core organizing model for simulation state.
- **Multi-backend rendering behind a common interface (RHI)**: **Metal on Apple, Vulkan
  elsewhere**, selected at build time. Both backends are written by us. (Was Vulkan-only;
  Metal added 2026-07-02 — see investigations/realtime-rendering/2026-07-02-metal-backend.md.)
- **C++23**, CMake build, dependencies vendored as git submodules under `external/`.

## Testing / entry points

- No application `main`. Instead, **driver tests** (under `tst/`) exercise subsystems
  directly — they are how we run and validate the engine during development, standing in for
  the consuming application.
- **Test architecture (2026-07-03)**: organized by **module then category** —
  `tst/<module>/<category>/*.cpp` where module ∈ {core, ecs, physics, graphics} and category ∈
  {unit, integration, benchmark, visual}. Each file **self-registers** cases via
  `TST_CASE(module, category, name) { ... }` (shared harness in `tst/harness/`; `TST_REQUIRE`
  throws + reports, no abort). CMake globs categories (`CONFIGURE_DEPENDS`) into **three
  executables** — no per-file wiring:
  - **`tests`** — unit + integration (headless pass/fail). Run all `./build/tst/tests`, filter
    `--module physics`, `--category unit`, or `<module>.<name>`; `--list` enumerates. **CTest**
    registers one entry per module (`ctest --test-dir build`).
  - **`benchmarks`** — benchmark suites (run in **Release**; print timings, not pass/fail).
  - **`visuals`** — interactive windowed demos (Apple); run one by name, e.g. `visuals rolling`.
  Adding a test = drop a `.cpp` in the right folder with a `TST_CASE` — it's picked up
  automatically. Graphics suites + all visuals are Apple-only (Metal backend).

## Constraints that follow from the goals

- **Headless operation is a first-class mode, not an afterthought.** ML training and
  offline rendering must run without a window or swapchain. Today the render loop is
  coupled to a GLFW window + swapchain (see architecture.md). This coupling has to be
  broken.
- **Throughput matters as much as latency.** ML training wants many environments stepped
  in parallel; the design should favor data-oriented layouts and batching (the existing
  `DrawJob` / instance-SSBO path is a good seed for this).
- **Determinism** is valuable for ML reproducibility — worth keeping in mind as the
  simulation and physics layers are built.
- **Compute, not just graphics.** ML and some rendering work belongs on compute pipelines
  and/or the compute queue; today only the graphics queue and graphics pipelines exist.

## Build strategy

- **Core first, then one graphics refactor pass.** Build `engine::core` to a good state
  standalone; do **not** refactor graphics incrementally alongside it. Once core is solid,
  go through the entire graphics package in one dedicated pass (refactor, delete, reorganize
  behind the RHI). Status (2026-07-03): the **RHI + Metal backend** landed as the current path
  (offscreen + windowed, instanced, per-instance materials); the **legacy Vulkan code is parked**
  under `src/graphics/vulkan/` (doesn't build on Apple — fine) awaiting the Vulkan-behind-RHI
  port. So "add Metal" is done; "port Vulkan behind the RHI" is the remaining half of the pass.

## Driving milestone: "a ball rolling down a plane"

The first end-to-end target — a long-term vertical slice that forces the core systems to come
together (geometry, physics, graphics, the RHI). Simple to *describe*, but deliberately
scoped to exercise the overall goals, not just draw one sphere:

- **The scene**: a sphere under gravity rolling down an inclined plane (physics integration +
  collision + a real camera/render).
- **Headless + deferred rendering**: run the same sim with no window, rendering to offscreen
  targets (and/or deferred), as ML training and offline rendering require.
- **Parallel simulations**: run many independent instances of the scene at once (training
  throughput), not just one.
- **Massive scale**: e.g. ~100,000 spheres, to pressure-test batched/instanced rendering,
  data-oriented storage, and the physics broadphase.

This milestone is the yardstick for "is core in a good state?" and defines what the graphics
refactor pass must ultimately support.

> **Status (2026-07-03)**: the **physics + sim slice is met** — a sphere rolls without slipping
> down an inclined plane through the full ECS → physics → sync → `scene::extract` → Metal
> Renderer stack (headless `tst/physics/integration/milestone.cpp` + windowed
> `tst/physics/visual/rolling.cpp`). The broader **capability targets are still pending**:
> a clean headless/offscreen *device* split + deferred rendering, parallel-world ML stepping,
> and the ~100k-sphere instanced-at-scale case. See architecture.md "Milestone status".

## Next milestone: "a physics humanoid walking on terrain" (RL-ready)

Full plan: investigations/physics/2026-07-03-humanoid-rl-milestone-plan.md.

> An **articulated, physically-simulated humanoid** (jointed limbs, actuated, affected by
> gravity/contact/friction) that can be **driven interactively (keyboard/mouse) or
> programmatically (an action vector)**, standing and walking about **procedural terrain**,
> rendered with **basic lighting + a controllable background**, and **steppable headless in
> parallel batches** exposing **batched observation/action tensors**.

This is the *engine-side* target: the mechanism and infrastructure that make an RL locomotion
task possible. It deliberately **stops short of training** — the RL algorithm, reward shaping,
curriculum, task definitions, Python bindings, and cloud orchestration live in a **downstream
repo** (see "Direction: engine ↔ simulation split"). The MuJoCo "Humanoid" / DeepMind Control
locomotion benchmarks are the mental model for scope. It forces the systems Milestone 1 left
open: articulated dynamics + actuation, non-flat (terrain) collision, input, lighting, and a
vectorized headless env interface. Phased plan (A input+graphics → B articulated physics →
C terrain → D env interface) is in the investigation doc; backlog items in todo.md.

Key decision (RESOLVED 2026-07-04): **articulation approach** — maximal-coordinate joint
*constraints* on the existing impulse solver (fast to a visible result, reuses everything) vs
**reduced-coordinate** articulation (Featherstone/ABA; what real humanoid-RL uses; differentiable-
friendly; a much bigger build). We took the recommended path and now have **both**: constraints
first, then a reduced-coordinate `PhysicsWorld` backend, plus a separate **differentiable** reduced
engine. See architecture.md (physics reduced/diff sections) + the reduced-coordinate/differentiable
investigation docs.

## Active milestone: "a small, performant outdoor arena scene"

Full plan: investigations/realtime-rendering/2026-07-23-outdoor-arena-milestone.md.

> A **small outdoor arena** — grassy ground with dirt patches + small rocks, a little local
> atmosphere, and a tree with **leaves and grass waving in the wind** — rendered well and
> **performantly**, built as a **separate game repo** that consumes this engine as a dependency.

This is the **first real game-on-engine**, and the first consumer to exercise the "engine ↔
application split" for rendering: the **engine provides mechanisms** (RHI, render graph, ECS,
materials, culling, asset loading) and the **game provides content + shaders** (the arena, its
textures/meshes, and the grass/wind/ground-blend pipelines). It pushes the renderer past colored
primitives into **textured, authored content** — the biggest missing piece — while staying small
and bounded so quality + performance can be chased without scope creep. It's deliberately an
**arena for (later) fighting characters**, so the static scene must leave a large frame budget for
animated, simulated characters on top. Non-goals here: the characters themselves (skinned mesh +
animation), non-flat terrain collision, networking.

## Direction: engine ↔ simulation split

This repo is the **engine** (a library). Once it has "enough" for the humanoid milestone
(input, lighting, articulated physics + actuators, terrain, and a generic `Environment`/
`VecEnv` mechanism with batched obs/action buffers), the **full simulation moves to a separate
repo** that pulls the engine in as a dependency and owns the task/reward/RL-algorithm/cloud
layers. Design consequence: keep the env/articulation/observation-action API **clean, stable,
plain-data, and free of task/policy assumptions** at the boundary.

## Non-goals (for now)

- Networking / multiplayer.
- Editor / tooling UI.
- Scripting layer.

(Revisit these once the core ECS + headless rendering + physics foundations exist.)
