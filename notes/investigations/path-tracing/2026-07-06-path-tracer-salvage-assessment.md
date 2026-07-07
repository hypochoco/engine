# Path tracer salvage — reuse assessment of path-hypochoco (2026-07-06)

Evaluation of an old student path tracer (`~/Projects/path-hypochoco`, a Brown CS1230-style offline
CPU path tracer) for reuse in the engine's planned `engine::pathtracer` head. What's usable, what
conflicts with our infrastructure, and the bugs to fix on the way in. Feeds the integration plan at
the bottom.

## What it is
Recursive Monte-Carlo path tracer. Infrastructure: **Qt** (QImage/QRgb output, QSettings `.ini`
config), **Eigen** (math), a vendored **BVH** (Brandon Pelfrey's, `Object*`-virtual + per-object
bbox), **CS123 XML scene parser** + **tinyobjloader** for loading. Core (`pathtracer.cpp`): mirror
(illum 5), Fresnel dielectric w/ Schlick (illum 7), diffuse + glossy (Phong) BSDFs, area-light NEE
(`directLightSampling`), Russian roulette, Reinhard tone map. Single-threaded (OpenMP commented out).

## Reuse verdict — drop the infrastructure, keep the algorithms

**Infrastructure — not reused (we have equivalents or standards that conflict):**
- **Qt** — we use GLFW + Metal + headless readback. Replace with our harness / `stb_image_write`.
- **Eigen** — the engine standardized on **glm** engine-wide. Decision (owner, 2026-07-06):
  **do NOT adopt Eigen for the path tracer** — port the (shallow) math to glm; a second linear-
  algebra library is not worth the constant conversions at every boundary. Eigen's real value is
  high-dimensional vectors/matrices + solvers for *geometry processing* work later; revisit adopting
  it **then**, scoped to that, not for the path tracer.
- **CS123 XML parser + `CS123SceneData`** — superseded by our ECS + `core::geometry` +
  `scene::extract`. Not reused.
- **tinyobjloader** — already vendored in `engine::core`. Duplicate.
- **BVH (Pelfrey)** — Eigen-templated, `Object*`-virtual, pointer-chasing (not data-oriented).
  Usable as a correctness reference only; permissive license (attribution) if we derive from it.

**BVH status in our engine (checked 2026-07-06):** we do **not** have a ray-traversal BVH. Physics
has *broadphase* only — `sweep_and_prune`, `uniform_grid`, `aabb` — for dynamic pair-culling, a
different problem than static ray-vs-triangle traversal (the "SAP/BVH is Phase 2" comments in
`sequential_impulse_world.cpp` are aspirational). So there's nothing to reuse directly. **Decision:
defer the BVH** — the CPU reference integrator (step 1) uses brute-force ray-vs-all-triangles, which
is fine for small Cornell-box correctness scenes. Build/choose a real BVH later (our own, data-
oriented over `core::geometry`, possibly sharing the physics `aabb` primitive).

**Algorithms — the real value (≈20% of the repo), portable after a glm port + bug fixes:**
the `traceRay` integrator, BSDF handling, area-light NEE, Fresnel/Schlick, Russian roulette,
Möller–Trumbore triangle intersection, hemisphere sampling, Reinhard tone map (we also already have
`tonemap.slang`). These slot into an `engine::pathtracer` head consuming the neutral `RenderView`
(geometry via `MeshHandle`, lights, camera) that `scene::extract` already produces.

## Bugs found (owner's writeup flagged "direct lighting may be slightly off")
1. **Direct-lighting estimator is wrong** (`directLightSampling`): sums per-sample radiance without
   each sample's area weight, then multiplies the total by the *summed* area `A` and divides by
   count `N` (`return A * radiance / N`). Correct area sampling weights each sample by its own
   area/pdf. Only coincidentally right when all emitters are equal-area — the "slightly off".
2. **Non-uniform triangle sampling** (same fn): averages three edge-interpolated points → biased to
   the centroid. Needs the `√r1` barycentric formula for a uniform sample.
3. **Biased hemisphere sampling** (`randomDirection`): rejection with `angle < π/2.125` clips the
   hemisphere but still reports `pdf = 1/(2π)` — pdf no longer matches the sampled domain → biased
   indirect lighting. Fix: cosine-weighted hemisphere sampling (and use the reported pdf honestly).
4. **`Ray::inv_d` is `.normalized()`** (`Ray.h`): the inverse direction must be component-wise
   `1/d`, not normalized — corrupts any slab/BBox test that uses it.
5. **No epsilon offset on secondary-ray origins**: reflection/refraction/shadow rays start exactly
   at `i.hit` → shadow acne / light leaks (Möller–Trumbore's `t > EPS` only partly masks it).
6. **`toSpherical` uses `atan(y/x)`** not `atan2` (wrong quadrant) — latent (appears unused).
7. **Single-threaded + `rand()`**: OpenMP commented out; `rand()` is global/non-thread-safe. Needs a
   per-thread/per-pixel RNG for parallelism.

Minor: `makeBRDF` picks diffuse-vs-glossy by magnitude (not a real layered material); refraction
attenuation `1/(1+d²)` is ad hoc (not Beer–Lambert).

## Integration plan (approved 2026-07-06)
1. **glm-ported CPU reference integrator** — a new `engine::pathtracer` head over `engine::rhi`.
   Take the integrator logic; drop Qt/Eigen/CS123/tinyobj-dup. Source geometry from
   `core::geometry` and camera/lights from `RenderView`. Output to an offscreen texture via readback.
   **Brute-force intersection (BVH deferred).** Fix bugs 1–7 above, validated against the retained
   Cornell-box ground-truth images. ← START HERE.
2. **Move the hot path to a Slang compute shader** over the RHI (megakernel first) — where an engine
   head needs to live.
3. **Add a real BVH** (data-oriented, over `core::geometry`; possibly shares physics `aabb`) — only
   once brute force is the bottleneck.
4. Wire it as a swappable head alongside `engine::render` (both consume `RenderView`).

Net: keep the integrator IP, port to glm, fix the estimator/sampling bugs, defer the BVH, and grow
from a CPU reference to a GPU compute head.
