# 2026-07-04 — Render Framework Plan (clustered forward+ render graph)

The mid-level rendering **framework** that sits between the ECS extraction and the RHI. The
training milestone is essentially met; the next push is modern realtime visuals (multiple
lights, shadows, AA, sky/atmosphere, post). Those all depend on a framework layer that does not
exist yet — today the renderer is a single hardcoded forward pass. This note designs that layer,
**worked backward from the performance goals**, and is deliberately backend-agnostic
(Metal today, Vulkan behind the same RHI later).

Builds on — does not repeat:
- [2026-07-02-rhi-interface-plan.md](2026-07-02-rhi-interface-plan.md): the RHI surface. §12.7
  explicitly **deferred the frame graph** until "deferred + offscreen + multi-view passes justify
  it." Multiple-lights-early + shadows + post now justify it — this note is that follow-up. §14 is
  the bindless plan (still the texture path).
- [2026-07-01-graphics-refactor.md](2026-07-01-graphics-refactor.md): why the renderer owns no
  scene data; the render-list direction.

Scope confirmed by owner (2026-07-04): **multiple lights early**; **grass + ray tracing deferred**;
invest in the **framework** first because everything else depends on it.

---

## 0. TL;DR (decisions)

1. **Lighting architecture: clustered Forward+** as the primary path, with the render graph kept
   **G-buffer-capable** so a deferred path can be added later for the offline/high-quality
   "rendering" workload. (§3)
2. **A `render::RenderGraph`** — passes declare resource reads/writes; the graph orders them
   (topological, deterministic) and inserts barriers from those declarations. **Explicit,
   correctness-first**: no automatic memory aliasing / pass-culling in v1 (added later as a pure
   optimization). This is the "render passes" abstraction. (§4)
3. **Fix two latent correctness bugs** in the current renderer as the first framework step: a
   per-frame buffer **hazard** (no ring buffering across frames-in-flight) and **multi-view
   clobber** (one instance buffer reused across views). A `FrameRingAllocator` + per-view
   sub-allocation. (§1, §4b)
4. **Two RHI additions** unblock all multi-pass and are the only backend-agnosticism risks:
   an explicit **barrier / resource-transition** primitive and a **`transient`** resource hint
   (maps to Metal memoryless / VK lazily-allocated). (§5)
5. **Test early / often + benchmark consistently.** Every step keeps the build green, has a
   headless pixel or numeric check, and — for anything on the hot path — a benchmark with recorded
   numbers. Parity anchor: `graphics.mesh` center rgba `154 31 32 255`. (§7, §8)
6. **Multithreading**: not now, but the graph is designed so parallel command recording and
   parallel-world rendering slot in without contract changes. (§9)

---

## 1. Current state (honest assessment)

`render::Renderer::render(frame, views)` is a single hardcoded forward pass in a loop over views.
Reading `src/graphics/common/renderer.cpp`, three things must change before any multi-pass work:

1. **No per-frame ring buffering (a real hazard).** `cameraUBO` / `instanceBuffer` /
   `materialBuffer` are single `CpuToGpu` buffers overwritten every frame, but
   `DeviceConfig::framesInFlight = 2`. Frame N overwrites them while the GPU may still be reading
   frame N-1. It has not bitten us because the tests render one frame and read back; it will bite
   the moment we sustain frames-in-flight.
2. **Multi-view clobber.** Within one frame every view uploads into the *same* instance/camera
   buffers; commands execute later, so all views read the **last** view's data. Multi-pass /
   shadow / multi-camera are inherently multi-view → this is load-bearing.
3. **No resource-state / barrier model.** `beginRendering` covers within-pass transitions; nothing
   expresses "pass B samples the texture pass A wrote." Undefined today because there is only ever
   one pass.

Also missing (greenfield, not bugs): transient targets (shadow/G-buffer/HDR), compute in the frame
loop, and a pass structure that isn't baked into `render()`.

The **RHI itself is in good shape** and already anticipates this work: MRT render targets, compute
pipelines + `dispatch`, `drawIndexedIndirect`, offscreen targets + readback, bindless registration,
blend presets, N-frames-in-flight config. The framework is built *on top of* it; the RHI only needs
the two additions in §5.

---

## 2. Requirements (backward from goals.md)

| Driver (goals.md) | Framework implication |
|---|---|
| **Multiple lights early** (owner) | Light-culling architecture (clustered) + a light-list data path, not one directional uniform. |
| Backend-agnostic (Metal/Vulkan) | Pass/resource/barrier model maps cleanly to both; no `MTL::`/`Vk*` above the RHI. |
| Throughput / many parallel envs | Zero per-frame heap allocation in steady state; graph compiled once, cheaply replayed. |
| ~100k instances (massive scale) | SoA instance ring + indirect-draw-ready + GPU culling as a compute pass. |
| Headless is first-class | Graph runs with no swapchain; terminal node is present *or* readback. |
| Determinism (ML reproducibility) | Pass order + resource identity are a deterministic function of the graph; extraction order already stable. |
| Compute, not just graphics | Cluster light-binning + culling are the first real compute consumers. |

---

## 3. Decision: clustered Forward+ (not deferred)

Enabling many lights forces the classic choice. **Clustered Forward+ is primary**; the graph stays
able to express a deferred G-buffer for later.

Forward+ divides the view frustum into a 3D grid of **clusters** (froxels: screen tiles × depth
slices). A compute pass bins each light into the clusters it touches. The opaque forward pass, per
fragment, finds its cluster and iterates only that cluster's light list. Many lights, one shading
pass, no fat G-buffer.

**Why forward+ over deferred, w.r.t. our constraints:**

| Axis | Forward+ | Deferred |
|---|---|---|
| **Backend-agnostic / Apple TBDR** | Uniform on Vulkan + Metal; no off-chip G-buffer → plays to Apple tile GPUs. | Efficient deferred on Metal wants *tile shading* (on-chip G-buffer) — a Metal-specific construct that doesn't map uniformly to Vulkan. Off-chip G-buffer is the bandwidth pattern TBDR punishes. |
| **Anti-aliasing** | Hardware MSAA is cheap → matters for AA now and thin geometry (grass) later. | MSAA is expensive/awkward in deferred. |
| **Transparency** | One shading path for opaque + transparent. | Can't shade transparents → needs a forward path *anyway*. |
| **Compute exercise** | Cluster binning is the compute-path seed we want regardless. | Similar. |
| **Many lights (100s+)** | Clustered scales well. | Historically deferred's edge; clustered forward closes most of the gap. |

Deferred still earns a place for the **offline/offscreen high-quality** goal (dense lights, decals,
SSR/GI). The framework commitment is therefore to **graph-expressible passes**, not to a hardcoded
"forward" or "deferred" pipeline — forward+ is simply the first pipeline we build on the graph.

---

## 4. Architecture

New mid-level layer in `engine::render`, between ECS extraction and the RHI.

```
        ┌──────────────────────────────────────────────────────────────────────┐
        │  ECS world  (Transform, RenderMesh, RenderMaterial, Light components)  │
        └──────────────────────────────────────────────────────────────────────┘
                                   │  scene::extract / extractViews   (stable order)
                                   ▼
        RenderView[] { view, proj, target, LightList, RenderItem[], InstanceData[], MaterialGPU[] }
                                   │   transient, per frame, plain data (no Vk*/MTL::)
                                   ▼
        ┌──────────────────────────── render::RenderGraph ───────────────────────────────┐
        │  passes declare reads/writes → topological order → auto-barriers → record       │
        │                                                                                 │
        │   FrameRingAllocator      per-frame-in-flight ring; per-view sub-allocation      │
        │                           (view constants | instance SoA | light list | mats)    │
        │   TransientResourcePool   graph-owned textures/buffers (shadow, HDR, clusters)    │
        │   PassRegistry            raster + compute passes, data-driven                    │
        └─────────────────────────────────────────────────────────────────────────────────┘
                                   │  rhi::CommandList (+ barrier primitive, §5)
                                   ▼
        ┌──────────────────────────── rhi::Device ───────────────────────────────────────┐
        │  owns all GPU objects (pools) · compile-time backend · dynamic rendering          │
        │        [ Metal backend ]        OR        [ Vulkan backend ]   (one compiled)      │
        └─────────────────────────────────────────────────────────────────────────────────┘
```

### 4a. The standard forward+ frame graph

```
   (per shadow-casting light / cascade)
        depth-only shadow pass ──────────────┐  writes: shadowMap[i]  (transient, depth)
                                              │
   cluster assignment (COMPUTE) ─────────────┤  reads: LightList, camera; writes: clusterGrid,
        bin lights into froxels              │          lightIndexList  (transient, storage)
                                              ▼
   opaque forward+ pass ──────────────────────  reads: shadowMap[*], clusterGrid, lightIndexList,
        instanced drawIndexed, per-cluster              instance/material buffers
        light loop, shadow sample             ─  writes: hdrColor (transient, RGBA16F) + depth
                                              ▼
   sky pass ───────────────────────────────────  writes: hdrColor (depth test against opaque)
                                              ▼
   transparent pass (forward, back-to-front) ──  reads: clusterGrid+lights; writes: hdrColor
                                              ▼
   post chain (COMPUTE or fullscreen) ─────────  reads: hdrColor; writes: ldrTarget
        tonemap → (AA) → (bloom)              ▼
   present (windowed)  |  readback (headless)    ← only this terminal node differs by mode
```

v1 implements only **opaque forward+ → present/readback** (plus the cluster compute + light
list). Shadows, sky, transparent, post are later graph nodes — each is additive, no contract change.

### 4b. Components

- **`RenderGraph`**: `addRasterPass(name, {reads}, {writes}, recordFn)` /
  `addComputePass(...)`; `compile()` (topo-sort by resource deps, deterministic tie-break =
  registration order; place barriers on read-after-write / write-after-read edges); `execute(frame)`
  (record into the RHI command list). Compiled structure is **cached** while the pass set is
  unchanged (steady-state = no per-frame graph rebuild → matches the throughput goal).
- **`FrameRingAllocator`**: one growable `CpuToGpu` arena per frame-in-flight; `alloc(bytes,
  align)` returns `{BufferHandle, offset}`. Per view, per frame we sub-allocate view constants /
  instance SoA / light list / materials. **Fixes §1.1 and §1.2 together.** `BufferBinding` already
  carries `offset` — the RHI supports this today.
- **`TransientResourcePool`**: allocates graph-declared textures/buffers by (desc, lifetime).
  v1 = allocate-per-frame from a cache keyed on desc (no aliasing). Later: alias
  non-overlapping-lifetime resources for memory savings.
- **`RenderView` growth**: replace the single `DirectionalLight` with a `LightList`
  (directional + point + spot as SoA plain data). Everything else on `RenderView` stays.

---

## 5. Backend-agnosticism review (must be bulletproof)

Multi-pass exposes exactly one real gap and a couple of nuances. Both map cleanly to Metal + Vulkan.

- **Barriers / resource state (the gap → RHI addition #1).** When a later pass samples an earlier
  pass's attachment: **Vulkan requires** an explicit `vkCmdPipelineBarrier` + image-layout
  transition. **Metal auto-tracks** hazards by default (`MTLHazardTrackingModeTracked`), so the same
  point needs little/none. Design: the **graph** derives the dependency from declared reads/writes
  and calls a new explicit RHI primitive — e.g.
  `CommandList::resourceBarrier(std::span<const ResourceTransition>)` with
  `{handle, fromState, toState}` where state ∈ {RenderTarget, ShaderRead, StorageRW, TransferSrc/Dst,
  Present}. Vulkan implements real barriers; Metal implements it as a near-no-op (and
  `useResource`/`updateFence` only where untracked/heap/argument-buffer resources need it). The
  *contract stays explicit* (so Vulkan is correct by construction) without penalizing Metal.
- **Transient / memoryless targets (→ RHI addition #2).** A resource the graph knows never leaves
  the frame maps to Metal **`MTLStorageModeMemoryless`** (stays in tile memory — big TBDR win for
  depth / MSAA / G-buffer) and Vulkan **`LAZILY_ALLOCATED`** transient attachments. Encode a
  `transient` flag on `TextureDesc` now: free correctness today, real perf lever later. (Add
  `TextureUsage::Transient` or a bool.)
- **Dynamic rendering**: already the model on both backends — no render-pass/framebuffer object
  graph to reconcile. ✓
- **Bindless** (plan §14): Metal argument buffer + residency; Vulkan descriptor indexing. Shadow
  maps + cluster light-list ride the same storage/bindless model so passes don't churn descriptors.
- **Compute↔graphics sync**: same barrier primitive covers the cluster-compute → forward-pass edge.

Net RHI delta: **(1) `resourceBarrier` on `CommandList`; (2) a `transient` texture hint.** Nothing
else the graph needs is missing.

---

## 6. Performance review vs. goals

- **Parallel envs / throughput**: graph compiled once + replayed; ring + transient pool pre-sized ⇒
  zero steady-state heap alloc. Envs that don't render never touch it. *Risk*: per-frame recompile —
  mitigated by caching the compiled graph when the pass set is unchanged.
- **100k instances**: unchanged model — geometry stored once, SoA instance ring, one instanced
  `drawIndexed` per bucket; a compute **cull → `drawIndexedIndirect`** node slots behind the same
  `RenderView` contract.
- **Headless**: terminal node is present *or* readback; nothing above knows the difference.
- **Determinism**: topological order with stable tie-break; extraction already stable ⇒ rendering
  stays a pure function of world state.
- **Compute**: cluster binning + culling are the first dispatches — exercises the never-used queue.
- **Apple TBDR**: forward+ + transient/memoryless targets avoid the off-chip-bandwidth trap.

Guardrail benchmark from day one (§8) so we *measure* rather than assume.

---

## 7. Testing discipline (test early / often)

Every step keeps the build green and adds a headless check:
- **Parity**: after the graph port, `graphics.mesh` / `graphics.scene` must still produce center
  rgba `154 31 32 255` (pixel-exact) — proves the graph is a faithful re-plumbing of the forward
  pass.
- **Ring/multi-view regression**: render 2 views in one frame + sustain 2 frames in flight; assert
  each view's pixels are correct (catches §1.1/§1.2). New `tst/graphics/integration/`.
- **Barrier smoke**: a 2-pass graph (pass A writes a texture, pass B samples it) reads back the
  expected value → proves the transition works headless. New `tst/graphics/unit/` or `integration/`.
- **Multi-light correctness**: place N point lights, verify shaded output matches a CPU reference at
  sample pixels within tolerance; verify a light outside a cluster contributes ~0.
- All headless (no window) so they run in the `tests` runner on CI.

## 8. Benchmarking discipline (benchmark consistently)

New `tst/graphics/benchmark/` suite (runs in the `benchmarks` runner, Release). Track, per change:
- **Frame record CPU time** vs. instance count (1k → 100k) — the graph/record path must stay flat.
- **Cluster-binning compute time** vs. light count (1 → 1k+).
- **Forward+ shading** cost vs. lights-per-cluster.
- Graph **compile** time (should be ~0 in steady state via caching).
Record numbers in the note / todo as they land, like the physics baseline did (700× broadphase etc.).

---

## 9. Multithreading opportunities (notes only — NOT now)

Rendering is *not* where we thread first, and definitely not in v1. But the graph is designed so it
can, without contract changes. Captured so the design doesn't foreclose it:

- **Parallel command recording (the natural fit).** Passes (or draw-list chunks within a big pass)
  are independent between barriers. Both backends support recording into multiple command
  buffers/encoders on multiple threads and submitting in order (Vulkan secondary/ multiple primary
  cmd buffers; Metal `MTLParallelRenderCommandEncoder` / multiple command buffers). The graph's
  explicit pass DAG is exactly the dependency information a job system needs to record independent
  passes concurrently. **When**: only once single-thread CPU record time is a measured bottleneck
  (the §8 benchmark tells us) — likely at high instance/pass counts, not before.
- **Parallel extraction.** The ECS `query().chunks()` path is already SoA/parallel-friendly;
  extraction of instance data could use `core::ThreadPool::parallelFor` per archetype chunk. Same
  determinism rule as physics (stable per-chunk order, concat in fixed order).
- **Parallel-world rendering (ML pixel obs).** The strongest lever, and it mirrors the physics win
  (7.7× on parallel *worlds*). Each env renders to its own offscreen target; N envs' frames recorded
  across the thread pool, one `Device`. The graph is per-view/target already, so this is "run the
  graph per world on a worker," not a redesign. **When**: if/when pixel-based RL needs throughput.
- **What NOT to thread**: the RHI `Device` resource pools + deferred-deletion queue (keep single
  submission thread or guard with a coarse lock; contention there kills the win). Cluster binning is
  already GPU-parallel (compute) — no CPU threading needed.

Guiding rule (same as physics): **thread across independent units (passes, worlds, chunks), never
inside a fundamentally serial submit.** Measure first.

---

## 10. Sequencing (each step: green build + test + — on hot paths — a benchmark)

1. **RHI additions** — `transient` texture hint + `resourceBarrier`; implement in Metal (barrier ≈
   no-op / `useResource` where needed); barrier smoke test. *Unblocks all multi-pass.*
2. **FrameRingAllocator + per-view sub-allocation** in the Renderer → fixes the frame hazard +
   multi-view clobber; regression test (2 views, 2 frames in flight).
3. **Minimal RenderGraph** (explicit order + auto-barriers, no aliasing); **port the current forward
   pass** onto it unchanged; headless pixel-parity test (`154 31 32 255`).
4. **Multi-light**: `RenderView` → light list; cluster-binning **compute** pass; forward+ shading in
   `mesh.slang`; N-light correctness test + binning/shading benchmark.
5. **HDR + tonemap** resolve node (sets up AA / sky / bloom).
6. Then, as additive graph nodes: **shadows** (CSM), **sky/atmosphere**, **AA** (MSAA or post).

Deferred (own tracks, per owner): grass/vegetation, ray tracing, full deferred G-buffer path,
multithreaded recording, memory aliasing in the transient pool.

---

## 11. Open items / to settle as we build

- Cluster grid dimensions + depth-slice distribution (exponential) — tune with the benchmark.
- Light list storage cap + overflow policy (clamp vs. spill) — pick a bounded, deterministic rule.
- Shadow atlas vs. array for CSM — defer to the shadow step.
- Whether post/tonemap is a fullscreen-triangle raster pass or a compute pass — measure both on
  Apple (compute avoids a raster pass but adds a barrier).
