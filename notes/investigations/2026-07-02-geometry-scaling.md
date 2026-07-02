# 2026-07-02 — Geometry Scaling & Ownership

How the current `engine::core` geometry types scale toward the milestone's massive-scale
target (~100k), and what it implies for cache locality and ownership. Analysis only — nothing
built here; conclusions feed todo.md.

Relates to: goals.md (massive-scale milestone), 2026-07-02-core-module-plan.md,
2026-07-01-graphics-refactor.md (shared buffers + instance path).

## Measured sizes (Apple clang, arm64)

| Type | size | notes |
|---|---|---|
| `Vertex` | 44 B (align 4) | vec3 pos + vec3 normal + vec2 uv + vec3 color; interleaved AoS |
| `Mesh` | 16 B | range descriptor (first/count vertex+index) |
| `Material` | 20 B | vec4 + int |
| `MeshData` | 48 B inline | two `std::vector` (24 B each) **+ heap payloads** |
| `ModelData` | 72 B inline | three vectors + heap payloads |

glm types are unaligned: vec3=12, vec2=8, vec4=16, mat4=64.

## Disambiguate the workload (critical)

- **100k *instances* of a few models** (the milestone's 100k spheres): geometry stored
  *once*; replicate only per-instance data. Sphere ≈ 561×44 B ≈ 24 KB once + 100k × mat4
  (64 B) = **6.4 MB** transforms. Scales beautifully → this is the **instanced/batched** path.
- **100k *distinct* geometries**: memory dominated by real vertex data. At ~5k verts/model,
  220 KB each → **~22 GB** of vertices. No layout trick fixes this — it's a
  streaming/LOD/compression problem; you don't keep them all resident.

For the milestone: instance one sphere 100k times. The interesting scaling is instance data +
a shared geometry store, not 100k `ModelData`s.

## Two layers (the current types only half-express this)

1. **`MeshData` — load-time, owning source format.** Each = 2 heap allocations. Fine as
   loader/generator output. **Footgun at scale** if used as the runtime store:
   - *Scattered allocations*: 100k distinct models × meshes × 2 vectors → hundreds of
     thousands of heap-scattered buffers → allocator pressure, fragmentation, TLB/cache
     misses on any bulk iteration.
   - *Value semantics bite*: `ModelData`/`MeshData` deep-copy all vertices on copy; a stray
     pass-by-value or a reallocating `vector<ModelData>` copies megabytes.
2. **`Mesh` ranges + shared GPU buffers — runtime store (intended target).** CPU keeps a
   *flat* `std::vector<Mesh>` (16 B → 100k = 1.6 MB, contiguous); vertex data lives in a few
   big GPU buffers. Cache-friendly, scales.

Takeaway: **`MeshData` is a transient loading format; don't hold 100k of them.** Upload into a
pooled store, then keep ranges/handles.

## Cache locality

- **AoS interleaved `Vertex` is correct for rendering** (GPU vertex fetch wants all attributes
  together). Don't SoA the render vertex.
- **Interleaved hurts attribute-selective CPU passes**: physics needing only `position`
  (12 B) still pulls the 44 B stride; culling wants bounds. Derive compact side data
  (position-only arrays, precomputed `AABB`/`Sphere` per mesh) instead of walking `Vertex`
  arrays on CPU hot paths.
- **Per-frame hot data = its own flat SoA**: `{ meshRange, materialIndex, transform }` in
  parallel arrays (render list / instance buffer), not reached by chasing
  `ModelData → MeshData`. (This is the `DrawJob`/instance-SSBO rework.)

## Ownership models

- **Current (value-owning `MeshData`)**: simple, RAII-correct, great for loading and small
  counts; not the authoritative store at scale.
- **Central geometry store + handles (scalable target)**: one `GeometryStore` owns pooled
  buffers (big vertex arena + index arena, or slab-allocated) and hands out `MeshHandle`s
  (generational index). Entities reference geometry by handle, never pointer. Benefits:
  contiguous memory, one bulk GPU upload, stable identity across add/remove (streaming), no
  per-model allocation storm.
- This is exactly the **deferred `memory/` `Handle`/`slot_map`** from the core plan. **~100k
  streaming instances/models is a legitimate trigger to un-defer it** — but build the minimal
  version *inside its first consumer* (the geometry store or the RHI resource registry), then
  promote to `core` only if a second consumer wants it.
- **CPU-retention policy**: at scale, don't keep CPU vertex copies after GPU upload. Retain
  only where needed (collision meshes, raycasting), ideally in a separate collision-geometry
  store — not by holding every `MeshData` resident.

## Vertex footprint

44 B is fine normally; at massive scale a compact GPU layout is worth it: position float3
(12) + normal oct-encoded (4) + uv half2 (4) + color RGBA8 (4) ≈ 24 B — roughly halves
bandwidth/VRAM. Keep `core::Vertex` full-precision as the *authoring* format; each backend
packs into a compact GPU layout. (Later, renderer concern.)

## Conclusions → todo

1. Keep `MeshData`/`ModelData` as **loader output only**; not the runtime store, don't hold
   100k of them.
2. Plan the runtime around a **central geometry store (pooled buffers) + generational
   handles**; entities hold handles. Treat ~100k as the trigger to build the handle/slot_map
   (inside its first consumer).
3. Drive the milestone's 100k spheres via **instancing** (one geometry, 100k transforms), and
   keep per-frame data in **flat SoA** render/instance arrays.
4. Derive **bounds / position-only data** for physics/culling rather than iterating
   interleaved vertices.
5. Note **vertex compression** and **distinct-geometry streaming/LOD** as later at-scale work.
