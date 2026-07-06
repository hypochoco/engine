# 2026-07-02 — `engine::core` Module Plan

Design for the backend-agnostic foundation module. `core` is what `graphics` (both Vulkan
and Metal backends), `physics`, and the future `ecs` all build on. It contains **no**
Vulkan, Metal, GLFW, or windowing code.

Ties into: goals.md (build-everything-ourselves), 2026-07-01-graphics-refactor.md (step 1),
2026-07-02-metal-backend.md (`core` stays backend-free while the RHI splits per backend).

> **Revision (2026-07-02): scope trimmed to avoid speculative bloat.** Removed `types.h`,
> glm aliasing (`math.h`), and `transform.h`. Shape primitives moved under `geometry/` and
> deferred. `memory/` (handles/slot_map) deferred — see §5 for rationale. Core for now is
> just **geometry + image + io**; everything else is "add when a real consumer needs it."
> Guiding rule: **YAGNI** — don't build shared abstractions before there's a consumer;
> promote to core only once ≥2 modules want the same thing.

---

## 1. Purpose & dependency rules

**Purpose**: shared data types and utilities with no GPU/backend/OS-window dependencies, so
they can be used identically on the Vulkan and Metal sides and by physics/ECS.

**May depend on**: the C++23 standard library, `glm` (used **directly**, not aliased), and —
only inside the `io/` loaders — `stb_image` and `tinyobjloader`. Nothing else.

**Must NOT depend on**: Vulkan, Metal/metal-cpp, GLFW, `engine::graphics`, `engine::physics`.
Dependencies point *into* core, never out. (core's `target_link_libraries` today is
`glm/tinyobjloader/stb` only — keep it that way.)

**Design stance**: data-oriented and value-semantic. Plain structs, contiguous storage,
minimal inheritance, no hidden allocations on hot paths.

---

## 2. Layout (current scope)

```
include/engine/core/
├── core.h                 # umbrella convenience include (optional)
├── geometry/
│   ├── vertex.h           # canonical pure-data Vertex (glm types used directly)
│   ├── mesh.h             # MeshData (CPU buffers) + Mesh (range descriptor)
│   ├── material.h         # minimal backend-agnostic material description
│   └── model.h            # ModelData: meshes + materials + submesh→material mapping
│   └── primitives.h       # makeQuad / makePlane / makeSphere generators
├── image/
│   └── image.h            # CPU Image: pixels + width/height + ImageFormat enum
└── io/
    ├── file.h             # read whole file → bytes (replaces Graphics::readFile)
    ├── model_loader.h     # OBJ → ModelData (tinyobjloader, moved out of graphics)
    └── image_loader.h     # file/memory → Image (stb, moved out of graphics)

src/core/  mirrors the above with .cpp for anything not header-only.
```

`glm` types (`glm::vec3`, `glm::mat4`, …) are used directly in these headers — no aliasing
layer.

---

## 3. Feature areas (with representative types)

### 3.1 `geometry/` — mesh/model data (the reason core exists)
Backend-agnostic, replacing the Vulkan-entangled structs in `graphics.h`:
```cpp
struct Vertex {              // pure data — NO Vk/Metal binding descriptions here
    glm::vec3 position;
    glm::vec3 normal;        // NEW vs current (loadObj had a "todo: normals")
    glm::vec2 uv;
    glm::vec3 color;
};
struct MeshData {            // CPU-side, owns its buffers
    std::vector<Vertex>    vertices;
    std::vector<uint32_t>  indices;
};
struct Mesh {                // range descriptor into shared GPU buffers (as today)
    uint32_t firstVertex, vertexCount, firstIndex, indexCount;
};
struct Material {            // description, not GPU handles — kept minimal for now
    glm::vec4 baseColorFactor {1,1,1,1};
    int       baseColorTexture = -1;   // index/ref, resolved by the backend (−1 = none)
};
struct ModelData {
    std::vector<MeshData>  meshes;
    std::vector<Material>  materials;
    std::vector<uint32_t>  meshMaterial;  // submesh → material index
};
```
- The **GPU vertex layout** (attribute/binding descriptions for Vulkan, `MTLVertexDescriptor`
  for Metal) is derived from `core::Vertex` *in each backend*, keeping this struct clean.
- Fixes the current `loadQuad` count bug and the missing normals by construction.
- `Material` stays deliberately minimal (base color + one texture ref); grow it only when the
  renderer actually needs more (metallic/roughness, normal maps, …).

### 3.2 `image/` — CPU image data
```cpp
enum class ImageFormat { R8, RG8, RGBA8, RGBA8_SRGB /* extend as needed */ };
struct Image { uint32_t width, height; ImageFormat format; std::vector<uint8_t> pixels; };
```
Decouples image loading/manipulation from GPU texture creation. Backends map `ImageFormat`
→ `VkFormat` / `MTLPixelFormat`. Absorbs the stb usage currently in `graphics.cpp` (R8
luminance path, solid-color generation).

### 3.3 `io/` — files & asset loaders
- `file.h`: `std::vector<std::byte> readFile(path)` (replaces `Graphics::readFile`; will also
  load compiled shader blobs — `.spv`/`.metallib` — for the RHI later).
- `model_loader.h`: `ModelData loadObj(path)` — tinyobjloader, moved out of `graphics_model`;
  produces backend-agnostic `ModelData` and can finally compute normals.
- `image_loader.h`: `Image loadImage(path)` / from-memory — stb, moved out of `graphics`.

---

## 4. Who consumes what (current scope)

| Core area | graphics (RHI/backends) | physics |
|---|---|---|
| geometry (mesh/model) | upload to GPU buffers/registries | collision meshes (positions/indices) |
| image | upload to GPU textures | — |
| io (file/loaders) | load shader blobs + assets | load collision assets |

---

## 5. Deferred — add only when a real consumer needs it

Recorded so the ideas aren't lost, but explicitly **not built now** (YAGNI). Each entry
notes the trigger that would justify creating it.

- **Shape primitives** (`AABB`, `Sphere`, `Ray`, `Plane`, `Frustum`) → live under
  `geometry/` when added. *Trigger:* physics broadphase/collision or renderer frustum
  culling. Shared by both, so a good core citizen — once one of them exists.
- **`Transform`** (TRS → matrix). *Trigger:* ECS transform component or the renderer's
  instance-matrix path. Small and high-use when it lands; premature now.
- **Math utility header** (non-aliasing helpers not in glm). *Trigger:* an actual utility
  function we need more than once. Until then, use glm directly; no empty header.
- **`memory/` — generational handles + slot_map.** *Assessment:* the generational-handle
  pattern is genuinely valuable long-run (stable IDs for GPU resources/entities, avoids
  use-after-free that raw indices don't). **But** building a generic container before a
  consumer exists means guessing the API. *Plan:* when the first consumer appears (RHI
  resource registry or ECS storage), build the minimal handle/storage it needs *there*;
  promote into core only when a **second** consumer wants the same thing. Do not build
  speculatively.
- **`diagnostics/`** (assert/log) — use `<cassert>` / `std::cerr` until we need leveled
  logging or richer asserts.
- **`time/`** (clock/fixed-timestep) — the consuming app owns the loop; add fixed-timestep
  helpers if/when deterministic sim stepping needs them (ML determinism goal).

---

## 6. Phasing

- **Phase 1 (all current scope):** `geometry/` (vertex/mesh/material/model), `image/`,
  `io/` (file + model_loader + image_loader). This is refactor step 1 — it lets us delete
  the mesh/model/image code from graphics and share geometry with physics. Independent of any
  backend, so it compiles/links on this (Apple) machine today.
- **Later:** pull deferred items out of §5 as their triggers arrive.

Suggested first commit: `geometry/vertex.h` + `mesh.h` (+ `core.h`), with a `tst/` driver
that builds a `MeshData`, proving the module compiles and links with no backend present.
Then `material.h`/`model.h`/`image.h`, then the `io/` loaders (which move real code out of
graphics).

---

## 7. Open questions

- **Namespace**: single flat `engine::` namespace (recommended — short call sites), or
  sub-namespaces? Headers are the organizing unit either way.
- **Vertex layout**: is `{position, normal, uv, color}` the canonical vertex, or do we want
  tangents (normal mapping) / bone weights (skinning) now? Adding fields later means touching
  shaders — worth a moment's thought before we commit.
- **Material richness**: minimal (base color + one texture ref) is proposed; confirm that's
  enough for the first renderer pass.
