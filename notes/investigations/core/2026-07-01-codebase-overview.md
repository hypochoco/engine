# 2026-07-01 — Codebase Overview

First pass over the WIP engine to understand its structure and where it stands relative to
the stated intent (ECS-based engine for realtime sim/games, ML training, and rendering,
with a Vulkan backend).

## Method / scope

- Read: top-level + per-module `CMakeLists.txt`, `.gitmodules`, `readme.md`,
  `include/engine/graphics/graphics.h` (full), `include/engine/physics/physics.h`,
  `src/physics/physics.cpp`.
- Structural/interface-level read of the graphics `.cpp` files (sizes + symbol overview);
  did **not** trace every line of the ~115 KB of graphics implementation.
- `notes/` was empty before this.

## Findings

1. **Solid, conventional C++/CMake skeleton.** C++23, split include/src, module libs
   (`engine::graphics`, `engine::physics`) under an `engine::engine` interface target.
   Deps vendored as submodules (glfw, stb, tinyobjloader) + `find_package` glm/Vulkan.

2. **Graphics is a mature-but-monolithic Vulkan setup.** One ~460-line `Graphics` class
   across 5 files, clearly evolved from vulkan-tutorial.com (MSAA, mipmaps, depth,
   validation layers, macOS portability). Beyond the tutorial, it has:
   - A **batched, data-oriented draw path**: `Mesh` ranges into shared vertex/index
     buffers, `DrawJob`s, and per-frame **instance-matrix SSBO**. Good seed for ECS.
   - A set of **reusable offscreen render helpers** in `graphics_custom.cpp`
     (render pass / framebuffer / pipeline / descriptor / granular command recording).
     Promising for headless rendering, though the main loop doesn't use them.

3. **Physics is a stub.** `Physics::test()` prints a string. Nothing else.

## Gaps vs. goals (the important part)

- **No ECS exists.** Despite being "structured around the ECS," there are no
  entity/component/system types. `DrawJob` + instance SSBO is data-oriented but has no
  entity layer feeding it. `MAX_ENTITIES = 16` is just a buffer cap.
- **No entry point / loop driver.** Library only; no `main`.
- **Rendering is coupled to a window + swapchain.** No headless device path — a problem
  for ML training and offline rendering, both of which should run windowless.
- **No compute path.** Graphics queue / graphics pipelines only; ML work will need
  compute.
- **No shaders in-tree, no SPIR-V build step**; pipelines take runtime shader paths.
- **No tests.**

## Conclusions → moved into core docs

- Established `notes/core/{goals,architecture,todo}.md` from this pass.
- Highest-leverage next steps (see `todo.md`): (1) ECS core, (2) app/loop layer,
  (3) break window/swapchain coupling for a headless mode. These unblock most of the rest.

## Open questions for the owner

- ECS approach preference: **archetype** (great iteration/cache behavior, good for the
  batched render path) vs **sparse-set** (simpler, flexible add/remove)?
- For ML training: in-process many-env stepping, or separate processes? Affects how
  headless + determinism are designed.
- Is `graphics_custom.cpp` intended to become the basis of an offscreen/offline renderer?
