# Anti-aliasing — MSAA + optional FXAA (RF6 AA)

Point-in-time investigation. Settles the design for anti-aliasing, the last item in the RF6
"shadows / sky / AA" trio. Folds into `core/` when done.

## Goal & guidance

Reduce edge jaggies (and, with FXAA, some shading aliasing) in the forward+ path. Same guidance as
the sky/atmosphere work: realtime, **lean toward performance**, test early/often, **benchmark
before AND after**. Scope (approved 2026-07-05): **hardware MSAA** (primary) + an **optional FXAA**
post pass. Both opt-in, **off by default** (parity anchors stay valid). TAA / MetalFX / SMAA
deferred.

Three aliasing sources, and which technique addresses each:
- **Geometric edges** (triangle silhouettes) — MSAA (coverage samples).
- **Shading aliasing** (HDR sun disc, bright point lights, future specular) — MSAA does NOT help
  (shading is per-pixel); FXAA (image-space) does.
- **Texture aliasing** — mipmaps; N/A (no albedo textures yet).

## Decision 1 — MSAA: hardware multisampling on the forward pass, resolve on-tile

The rasterizer takes N coverage samples/pixel but shades once/pixel/triangle; N color+depth samples
are stored and **resolved** (averaged) at pass end. Natural fit for **forward** rendering (our
path), and on Apple **TBDR** the samples live in tile memory and resolve on-tile — near-free, and
the MSAA buffer can be memoryless. Backend-agnostic (Metal + future Vulkan both expose sample
counts). Default **4×** (quality/cost sweet spot; 2×/8× selectable).

Mechanics: the forward pass renders into an **MSAA color + MSAA depth** target and **resolves** the
color into the existing single-sample HDR target (or the swapchain when tonemap is off). The rest
of the graph is unchanged — tonemap samples the resolved HDR exactly as today.

## Decision 2 — RHI additions (the fields exist; the Metal backend ignores them today)

`TextureDesc.sampleCount` and `GraphicsPipelineDesc.sampleCount` already exist (default 1) but are
unused in the backend. Add:
- **MSAA textures**: honor `sampleCount` → `MTLTextureType::Type2DMultisample` + `setSampleCount(n)`
  (guard with `supportsTextureSampleCount`).
- **Pipeline**: `createGraphicsPipeline` → `setRasterSampleCount(desc.sampleCount)`.
- **Resolve**: the render graph `ColorTarget` gains a resolve target (`resolveTex`/`resolveRT`).
  When set, the backend binds the MSAA texture as the attachment `texture`, the single-sample
  texture as `resolveTexture`, and `storeAction = MultisampleResolve`. Depth attachment is MSAA
  (sampleCount n), store DontCare.

A pipeline's `rasterSampleCount` must match the attachment sample count, so **the app must build its
mesh + sky pipelines with `sampleCount = n`** (same pattern as HDR needing RGBA16F pipelines — the
engine builds no pipelines). Sky is drawn inside the forward pass, so it must be MSAA-compatible
too. Documented on `setMSAA`.

## Decision 3 — HDR × MSAA resolve caveat

Fixed-function resolve averages samples **linearly**; a single very bright HDR sample (the sun) at
an edge can bias the average hot. v1 uses the **fixed-function** resolve (simple, TBDR-cheap) — the
subsequent tonemap roll-off hides most of it. A **custom tonemap-weighted resolve** shader
(Karis-style luma weighting, resolving MSAA samples with `Texture2DMS`) is the quality upgrade,
**deferred**.

## Decision 4 — FXAA: optional cheap post pass (independent of MSAA)

`fxaa.slang`: a fullscreen pass, **after tonemap**, on the LDR image. Compact luma-FXAA (our own
implementation, credited to Lottes' FXAA): compute luma, find local edge contrast, and blend along
the edge direction. One pass, no extra buffers beyond an intermediate LDR texture, backend-agnostic.
Catches the **shading aliasing MSAA misses** (the sun disc), at the cost of a slight overall blur.

Ordering with FXAA on: forward(±MSAA) → resolve → **tonemap → intermediate LDR texture** → FXAA →
present. (Today tonemap writes straight to the swapchain; with FXAA it writes to a sampleable LDR
texture that FXAA resolves to the swapchain.) `Renderer::setFXAA(pipeline, sampler)` opt-in; app
builds the fullscreen fxaa pipeline (LDR/swap format). Register `fxaa.slang` in the shader list.

MSAA and FXAA are independent: either alone or both (MSAA cleans geometry, FXAA cleans the sun disc
+ residual).

## Decision 5 — testability (`graphics.aa`, headless)

Aliasing is measurable at edges. Render a high-contrast **diagonal edge** (a rotated quad/triangle)
over a contrasting background at modest resolution; count pixels whose value is *strictly between*
foreground and background (partial-coverage / blended pixels):
- **MSAA on** ⇒ many more intermediate edge pixels than off (edge smoothed); interior + background
  unchanged.
- **FXAA on** ⇒ likewise increases intermediate pixels along a rendered hard edge.
- **Parity**: both off ⇒ identical to today (assert a parity anchor / interior pixel).

## Decision 6 — benchmark before/after

`render_graph`: MSAA off vs 4× (frame time — expect small on TBDR), FXAA off vs on (one fullscreen
pass). Record next to the RF6 entry.

## Sequencing

1. RHI MSAA (sampleCount textures + pipeline rasterSampleCount + resolve store action) — a barrier/
   smoke build.
2. `Renderer::setMSAA` + MSAA color/depth targets + resolve wiring.
3. `graphics.aa` MSAA edge test; benchmark MSAA off vs 4×. **Land MSAA fully green first.**
4. `fxaa.slang` + `Renderer::setFXAA` + the tonemap→intermediate→FXAA ordering.
5. FXAA edge test; benchmark FXAA off vs on.
6. Wire MSAA+FXAA toggles into a visual demo; update core notes.

## Deferred

TAA (motion vectors + history + reprojection + neighborhood clamp), MetalFX temporal, SMAA, custom
tonemap-weighted HDR MSAA resolve, alpha-to-coverage for MSAA'd transparency, per-sample shading.
