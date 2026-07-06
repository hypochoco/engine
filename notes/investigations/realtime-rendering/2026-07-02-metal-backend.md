# 2026-07-02 — Metal Backend / Multi-Backend Review

Goal: in addition to Vulkan, support a Metal backend, building Metal on Apple and Vulkan
elsewhere, behind a common interface. This reviews the current state and what that takes.

Scope of read: top-level + all module `CMakeLists.txt`, `.gitmodules`, `.vscode/tasks.json`,
`tst/`, the new `core` stubs, `external/metal-cpp` (README + header layout), and re-check of
`graphics.h` coupling.

---

## 1. Where things stand (your changes since the refactor report)

Done:
- **`engine::core` module** scaffolded (`core/CMakeLists.txt`, empty `src/core/core.cpp`,
  empty `include/engine/core/core.h`). Links glm/tinyobjloader/stb. Meta target now links
  `engine::core` too.
- **CMake modernized**: `GLOB_RECURSE` for headers + sources incl. `*.mm`, `source_group`
  for IDE tree, per-module include dirs.
- **Platform backend selection** in the root `CMakeLists.txt`:
  `if(APPLE)` → `find_library(Metal)`, `find_library(QuartzCore)`; `else()` →
  `find_package(Vulkan)`. Graphics links Metal+QuartzCore on Apple, Vulkan otherwise.
- **`external/metal-cpp` submodule** added (Apple's header-only C++ Metal bindings; note it
  includes the new **Metal 4 / MTL4** API surface as well as classic MTL).
- **`tst/`** module with a placeholder `test` executable.

Gaps / issues in the current setup (before any Metal code is written):

1. **The existing graphics code is 100% Vulkan and will not compile on Apple now.**
   `graphics.h` does `#define GLFW_INCLUDE_VULKAN` and exposes `Vk*` types as public members;
   all five `.cpp` files call Vulkan directly. With `find_package(Vulkan)` gone on Apple,
   this can't build there. So Metal isn't an "add-on" — it forces the backend abstraction.
2. **`GLOB_RECURSE` will feed Vulkan *and* Metal sources to both platforms.** Once
   `src/graphics/metal/*.mm` and `src/graphics/vulkan/*.cpp` coexist, each platform will try
   to compile the other's files. Need per-backend source selection in CMake (glob a backend
   subdir chosen by platform), not one recursive glob.
3. **`OBJCXX` language not enabled.** `project(engine LANGUAGES CXX)` — CMake needs
   `OBJCXX` enabled (guarded on Apple) for `.mm` files to compile.
4. **`Foundation` framework not linked.** metal-cpp uses `NS::` (Foundation) heavily;
   Metal + QuartzCore alone won't link. Need `find_library(FOUNDATION_FRAMEWORK Foundation)`.
5. **metal-cpp include dir not added** to any target's include paths.
6. **No translation unit defines the metal-cpp implementation macros.** Exactly one `.cpp`
   must define `NS_PRIVATE_IMPLEMENTATION` + `MTL_PRIVATE_IMPLEMENTATION` (+
   `CA_PRIVATE_IMPLEMENTATION` for QuartzCore) before including the headers, or linking fails.
   (Confirmed from metal-cpp README "Adding metal-cpp to a Project".)
7. **`test` target name** collides with CTest's reserved `test` target and links no engine
   code yet. Minor, but rename (e.g., `engine_tests`).

---

## 2. The key realization: "add Metal" == "do the refactor"

You cannot bolt Metal onto the current `Graphics` class — it *is* the Vulkan backend, with
`Vk*` handles as public state that the rest of the engine will reference. Supporting two
backends **requires** the abstraction layer the refactor already proposed (Device /
Swapchain / Pipeline / CommandBuffer / Resources). In other words:

> The multi-backend goal and the graphics refactor are the **same effort**. Define a
> backend-agnostic interface (an "RHI" — Render Hardware Interface), then implement Vulkan
> and Metal behind it. `engine::core` stays backend-free; `engine::graphics` becomes
> "RHI interface + one impl per backend."

This reframes the refactor sequence: step 3 ("first-class pipelines") and the Device split
become the definition of the RHI, and Metal is a second implementation of it.

---

## 3. Strategic fork: build your own RHI vs. adopt one

Worth an explicit decision before investing, because hand-writing and maintaining two
correct backends (plus shaders, sync, memory) is a large, ongoing cost.

**A. Build your own thin RHI** (what the phrasing "a graphics library for metal" implies).
- Pros: full control, no big dependency, exactly the abstractions the engine wants, great
  learning, tailored to ECS/ML/headless goals.
- Cons: large surface area; you own two backends + a shader pipeline forever; easy to get
  sync/memory subtly wrong (already saw semaphore/`vkQueueWaitIdle` issues in the Vulkan
  code).

**B. Adopt an existing abstraction** and drop the hand-rolled Vulkan:
- **Dawn (WebGPU)** or **wgpu** — WebGPU C API over Vulkan/Metal/D3D12; compute + headless
  are first-class; strong fit for the ML/rendering/sim goals; single shader language (WGSL,
  or via Slang/Tint). Heavier dependency, opinionated.
- **SDL3 GPU** — newer, clean C API over Vulkan/Metal/D3D12; compiles shaders offline;
  lighter than Dawn.
- **bgfx** — mature, many backends; older-style API; its own shader toolchain (shaderc).
- **Diligent Engine** — full-featured RHI, closer to a rendering framework.

Recommendation to weigh: given the goals list (Vulkan + Metal + compute + headless + ML),
an existing RHI (Dawn/WebGPU or SDL3 GPU) would deliver Metal "for free" and save the
biggest hidden cost (shaders + two backends). If the intent is partly to *learn* graphics
programming or to keep zero heavy deps, build your own. This is a genuine fork — decide it
deliberately; the rest of this doc assumes **build-your-own** since that matches the ask.

> **DECIDED (2026-07-02): build our own.** The purpose of the project is to develop these
> systems in-house — no third-party RHI. Option B is recorded only for context. Both the
> Vulkan and Metal backends are hand-written; the shader-toolchain cost (§5) is accepted.

---

## 4. If building your own RHI — interface shape

Backend selected at **compile time** (one backend per binary, as the CMake already implies),
so runtime virtual dispatch is optional. Two viable styles:

- **Runtime polymorphism**: `rhi::Device` etc. as abstract interfaces, one impl per backend.
  Simplest to maintain; vtable cost is negligible at pipeline/pass granularity if the API is
  coarse (avoid per-draw virtual calls — batch/record instead).
- **Compile-time**: a single `rhi::Device` type whose impl is chosen by CMake (pimpl or a
  typedef to the active backend). Zero overhead, but more template/build machinery.

Either way the interface is roughly (backend-agnostic types only — no `Vk*`/`MTL::` leak):

```
rhi::Instance / rhi::Device        // create, pick GPU, queues
rhi::Swapchain (optional)          // windowed present target
rhi::Buffer / rhi::Texture / Sampler
rhi::ShaderModule                  // from precompiled backend blob (SPIR-V / metallib)
rhi::Pipeline  (+ PipelineConfig)  // the builder from the refactor report
rhi::CommandBuffer / encoders      // begin/bind/draw/dispatch/end
rhi::Fence / rhi::Semaphore        // frame sync
```

`PipelineConfig` from the refactor report maps cleanly onto both backends (see table below).

### Vulkan ↔ Metal concept mapping

| Concept | Vulkan | Metal (metal-cpp) | Notes |
|---|---|---|---|
| Device | VkInstance + VkPhysicalDevice + VkDevice | `MTL::Device` | Metal has no instance/physical split — simpler |
| Queue | VkQueue | `MTL::CommandQueue` | |
| Command buffer | VkCommandBuffer (+ pool) | `MTL::CommandBuffer` (transient, from queue) + encoders | Metal has no long-lived pool |
| Encoders | vkCmd* on one CB | `MTL::RenderCommandEncoder` / `Blit` / `Compute` | Metal splits by pass type |
| Swapchain | VkSwapchainKHR + images | `CA::MetalLayer` + `CA::MetalDrawable` | Needs QuartzCore + window glue |
| Render pass / FBO | VkRenderPass + VkFramebuffer | `MTL::RenderPassDescriptor` | Metal bakes load/store/clear in; like VK dynamic rendering |
| Pipeline | VkPipeline (monolithic) | `MTL::RenderPipelineState` + `DepthStencilState` + encoder raster state | Split across a few objects |
| Descriptors | sets / layouts / pool | argument buffers, or `setBuffer/Texture` (MTL3) / MTL4 argument tables | Binding models differ most here; bindless is natural in Metal |
| Memory | VkDeviceMemory + findMemoryType | storage modes: Shared/Managed/Private | Metal is much simpler; VMA is Vulkan-only |
| Sync | VkFence / VkSemaphore | `MTL::Fence` / `MTL::Event` + completion handlers | Present via drawable |
| Shaders | SPIR-V | MSL source or `.metallib` | The big one — see §5 |

Designing the Vulkan side to use **dynamic rendering** (`VK_KHR_dynamic_rendering`) makes
render-pass/framebuffer handling line up much more closely with Metal.

---

## 5. Shaders are the biggest hidden cost

Vulkan consumes **SPIR-V**; Metal consumes **MSL** or a compiled **`.metallib`**. Today
pipelines read raw `.spv` at runtime — that has no Metal equivalent. Options:

- **Author once, cross-compile in the build** (recommended):
  - **Slang** → emits SPIR-V *and* MSL/metallib from one source. Modern, well-supported.
  - or GLSL/HLSL → SPIR-V (glslang/dxc) → **SPIRV-Cross** → MSL → `metal`/`metallib`.
- Abstract `rhi::ShaderModule` over "a backend blob": `.spv` for Vulkan, `.metallib` (or MSL
  compiled at runtime via `MTL::Device::newLibrary`) for Metal.
- This pulls the still-open "shaders in-tree + build step" TODO to the front — now it must
  also target two output formats. Decide the shader language/toolchain early; it colors the
  whole pipeline API.

---

## 6. metal-cpp integration specifics (grounded in its README)

- Header-only, **C++17+**, pure C++ — Metal calls themselves work from `.cpp` (no ObjC).
- **Exactly one TU** must define `NS_PRIVATE_IMPLEMENTATION` + `MTL_PRIVATE_IMPLEMENTATION`
  (+ `CA_PRIVATE_IMPLEMENTATION` for QuartzCore) before including, to emit symbols.
- Link **Foundation + Metal + QuartzCore** frameworks (Foundation currently missing).
- **Reference counting is manual** (no ARC): objects from `alloc`/`new`/`Create` start at
  retainCount 1; use `release()` or `NS::SharedPtr` (`NS::TransferPtr`/`RetainPtr`). Wrap in
  RAII on our side to avoid leaks.
- **AutoreleasePool per frame**: create/drain an `NS::AutoreleasePool` around each frame's
  work (the render loop), or temporaries leak.
- **Window glue needs a small `.mm` shim**: GLFW is created with `GLFW_NO_API` (already the
  case — good, works for both backends). For Metal, attach a `CA::MetalLayer` to the GLFW
  `NSWindow` via `glfwGetCocoaWindow` (`#define GLFW_EXPOSE_NATIVE_COCOA` + `glfw3native.h`);
  setting `contentView.layer` is an ObjC message, so isolate it in one Objective-C++ `.mm`.

### Concrete CMake changes needed

- `project(engine LANGUAGES CXX)` → add `OBJCXX` (guarded on Apple, or
  `enable_language(OBJCXX)` inside `if(APPLE)`).
- `find_library(FOUNDATION_FRAMEWORK Foundation REQUIRED)`; link it in the Apple branch.
- Add `external/metal-cpp` to graphics include dirs (Apple only).
- Replace the single `GLOB_RECURSE` over `src/graphics` with **per-backend selection**:
  compile `src/graphics/vulkan/**` when not Apple, `src/graphics/metal/**` (+ the `.mm`
  shim) when Apple, plus shared `src/graphics/*.cpp`. Prefer explicit backend subdirs.
- Rename `test` target (avoid CTest's reserved name); link it against `engine::engine`.

---

## 7. Suggested sequencing (folds into the refactor plan)

1. **Land `engine::core`** first (backend-agnostic Vertex/Mesh/Material/loader). Independent
   of backend; unblocks everything and compiles on Apple today.
2. **Define the RHI interface** (headers only) from the refactor's Device/Pipeline/etc.
   design — no `Vk*`/`MTL::` in the public surface.
3. **Reorganize the current Vulkan code** into `src/graphics/vulkan/` behind the RHI (this
   is the refactor's decomposition, done backend-side).
4. **Fix CMake** for real per-backend selection + Apple frameworks + OBJCXX + metal-cpp impl
   TU (§6).
5. **Pick the shader toolchain** (§5) and add the build step emitting SPIR-V + metallib.
6. **Implement the Metal backend** incrementally: Device → Swapchain (layer/drawable) →
   Buffer/Texture → Pipeline → command encoding → first triangle → parity with Vulkan path.
7. Keep a **headless** path in mind for both (ML/offline) — Metal offscreen is a texture +
   blit/readback, no layer needed.

---

## 8. Open questions for the owner

- **Build-your-own RHI vs. adopt (Dawn/WebGPU, SDL3 GPU, bgfx)?** The single most important
  decision — it determines whether Metal is weeks of work or mostly free. (§3)
- **Shader strategy**: Slang (one source → SPIR-V + MSL) vs. GLSL + SPIRV-Cross? (§5)
- **RHI dispatch**: runtime-virtual (simpler) vs. compile-time (zero overhead)? (§4)
- **Metal API level**: target classic `MTL` (widest macOS support) or lean into **MTL4**
  (present in the vendored metal-cpp, newer, closer to Vulkan's explicit model)?
- **Windows/D3D12 later?** If ever, that argues harder for adopting an RHI now vs. writing a
  third backend by hand.
