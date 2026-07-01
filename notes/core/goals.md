# Goals

## What the engine is for

A single engine serving three workloads on shared foundations:

1. **Realtime simulation / games** — interactive framerates, windowed presentation.
2. **Machine learning training** — likely headless, high-throughput, many parallel
   environments, deterministic where possible.
3. **Rendering** — offscreen/offline image generation (higher quality, not necessarily
   realtime).

## Architectural commitments

- **Entity Component System (ECS)** as the core organizing model for simulation state.
- **Vulkan** as the rendering backend.
- **C++23**, CMake build, dependencies vendored as git submodules under `external/`.

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

## Non-goals (for now)

- Networking / multiplayer.
- Editor / tooling UI.
- Scripting layer.

(Revisit these once the core ECS + headless rendering + physics foundations exist.)
