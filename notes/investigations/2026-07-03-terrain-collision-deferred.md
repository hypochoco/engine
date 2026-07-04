# Terrain / heightfield collision — DEFERRED (revisit)

Point-in-time note (2026-07-03). Milestone 2 "Phase C" is **deferred**; this preserves the
design and the reasoning so we can pick it up without re-deriving it. Living status:
`notes/core/todo.md` (Phase C marked deferred) + the milestone plan
([2026-07-03-humanoid-rl-milestone-plan.md](2026-07-03-humanoid-rl-milestone-plan.md)).

## Decision

Defer terrain. For the initial walking-humanoid milestone a **flat `Plane`** (already
implemented, stable resting + friction) is enough to get balance + locomotion working. Varied
terrain is a refinement of "walk about terrains," not a prerequisite for it.

**Revisit when** locomotion needs slopes / stairs / gaps / rough ground — e.g. a terrain
curriculum for RL, or testing robustness of a trained gait. Sequence-wise: do **Phase B
(articulation)** and **Phase D (env interface)** on flat ground first; slot terrain back in
before the milestone's "terrains" clause is truly exercised.

## Preserved Phase C design (grounded in the current collision code)

A new collider only needs: (1) a broadphase AABB, (2) inertia (irrelevant — terrain is static),
(3) narrowphase routines emitting `Contact`s. Everything downstream — sequential-impulse solver,
friction, CCD speculative margin, graph-colored parallel solve, determinism — then works
unchanged. Static terrain is never written by the solver, so it's race-free in the parallel
path for free (exactly like the existing infinite `Plane`).

- **Shape/plumbing**: add `struct Heightfield` to `shapes/shapes.h`; `ColliderDesc::Type::
  Heightfield` + member in `world.h`; `collision/heightfield.{h,cpp}` for the narrowphase.
- **Broadphase**: a heightfield is a *bounded* static shape — like `Plane` but finite in XZ and
  spatially varying. Slot it into the backend's "static, tested vs finite bodies" path (the
  `planeIdx_` route), but with an **AABB reject**: terrain world AABB = XZ extent × [minH, maxH];
  only bodies whose AABB overlaps it run the local narrowphase (avoids the plane's "test every
  body").
- **Narrowphase**: map the body's AABB → covered grid cells; run shape-vs-cell-surface per cell.
  Stage it:
  - **C1a Sphere vs heightfield** — closest point on the covered cell surface vs the sphere.
    Fast, low-risk, proves the integration + gives an immediate "ball on rough terrain" demo.
  - **C1b Capsule vs heightfield** — needed for the humanoid; per-cell contacts reusing the
    existing `gjk_distance`. Box can follow or stay deferred.
- **Internal-edge pitfall**: adjacent cells share edges; a body crossing one can catch a
  spurious normal and jitter. Mitigate by using the heightfield's **analytic bilinear-patch
  surface normal** (from the grid) rather than raw per-triangle normals — build this in from the
  start (matters for a stable walking gait).
- **Generation (`core::geometry`)**: `HeightfieldData` (height grid + spacing + origin) +
  generators (flat / ramp / stairs / value or Perlin fBm) + `makeHeightfieldMesh` → `MeshData`
  with finite-difference normals for shading. Shared: physics reads the samples, render gets the
  mesh (no asset files; consistent with `primitives`).
- **Height-data ownership**: inline `std::vector<Real>` in the shape is fine for one static
  terrain; switch to a non-owning pointer/handle to a shared `HeightfieldData` if grids get large
  or streamed.
- **Render/tests**: terrain is just an entity (`Transform` + static `RigidBody` w/ heightfield
  collider + `RenderMesh`) — no new bridge. Headless test: drop a sphere on a slope, assert it
  rests at the surface height (mirrors the milestone test). Visual: bodies dropped on noisy
  terrain + fly camera.

**Out of scope even when we resume**: overhangs/caves (heightfields can't represent them),
terrain LOD/streaming, GPU tessellation, per-region terrain materials.

## Related: do GJK/EPA already give us arbitrary triangle-mesh colliders? (No)

GJK/EPA operate through a support function and are correct only for **convex** shapes.
- **Convex meshes: already supported.** Feed the vertices as `ConvexHull`; GJK + EPA +
  `polytopeManifold` collide + stack them correctly today.
- **Concave/arbitrary meshes: NOT supported.** Feeding a concave mesh's vertices as one
  `ConvexHull` collides against its **convex hull** (a bowl acts like a solid dome). A real
  arbitrary-mesh collider needs an extra layer on top of GJK/EPA:
  1. **Convex decomposition** (e.g. V-HACD) → convex pieces, GJK/EPA each; or
  2. **Per-triangle collision** — each triangle as a degenerate convex, tested against nearby
     triangles. Requires a **BVH/midphase** over the mesh (we only broadphase *body* AABBs; a
     triangle soup has no useful single AABB), robust EPA on **zero-thickness** simplices (flat
     Minkowski difference is degenerate/fragile), **internal-edge** normal filtering, and
     manifold reduction across many triangle hits.
- Also: our `SupportShape::hull` support is a linear O(n) vertex scan (no hill-climbing), so a
  large hull is slow regardless — fine for small hulls, not for a big mesh.

So a **heightfield is the structured, tractable special case** of mesh collision (implicit grid
→ no BVH; analytic surface normal → little internal-edge pain), which is why we'd do it before a
general concave-mesh collider. A general static triangle-mesh collider (decomposition or
per-triangle + BVH + edge filtering) remains a separate, larger future item if overhangs /
arbitrary static geometry are ever needed.
