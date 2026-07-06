#  Graphics configuration system — review + plan (2026-07-05)

Centralize the renderer's scattered per-feature toggles + tuning knobs into one value-type config,
mirroring the physics config system (`2026-07-04-physics-config-system.md`). Review requested after
the RF6 shadows/sky/fog/AA additions multiplied the `Renderer::setX()` surface.

## Problem — knobs + toggles scattered across 7 setters + 2 buried constexpr blocks
Each feature is wired through its own setter, and each setter mixes **two disjoint concerns**:
GPU resource handles (which the app must supply — the engine builds no shaders/pipelines) and
plain-data tuning params + an implicit enable.

- `setTonemap(pipeline, sampler)` — HDR; enable = pipeline valid; no tunable params (curve in shader).
- `setClusterBinning(pipeline)` — clustered+; enable = pipeline valid; grid `CX/CY/CZ/MAX_PER` is a
  **file-scope `constexpr`** (`cluster::`), un-tunable without a recompile.
- `setShadows(pipeline, sampler, orthoHalfExtent, depthRange, bias)` — enable = pipeline valid;
  params in `Impl`; `shadow::MAP_SIZE` is a **file-scope `constexpr`**.
- `setSky(pipeline)` + `setSkyColors(zenith, horizon, ground, sunColor, sunAngularRadiusDeg,
  glowExponent, glowStrength, brightness)` — 8 palette/sun knobs held as `Impl` defaults.
- `setFog(density, heightFalloff, baseHeight, color, inscatterColor, inscatterExponent)` — no
  pipeline (in-shader); enable inferred from `density > 0`.
- `setMSAA(sampleCount)` — `sampleCount==1` = off; no renderer resource, but the app's mesh+sky
  pipelines must be built with a matching `GraphicsPipelineDesc.sampleCount`.
- `setFXAA(pipeline, sampler)` — enable = pipeline valid.

Problems (same shape as the physics review): ~20 tuning knobs reachable only through setters + 2
buried `constexpr` blocks (shadow map size, froxel grid) that need a source edit to change; **"is a
feature on" is implicit in resource validity / `density>0`**, so there's no clean "configured but
off" state (this is exactly what forced `atmosphere_scene` to build dual pipeline sets + a hand-
rolled `applyAA()`); no single place to see/diff/preset/record the graphics setup.

## Locked decisions (approved 2026-07-05)
1. **Value-type config, no global singleton** — same as physics. "Centralized" = one struct
   definition + one defaults source + one serializer, not one shared instance.
2. **Split data from resources.** Graphics' wrinkle vs physics: the engine builds no pipelines, so
   resource *handles* must stay app-provided and can't have data defaults. → `GraphicsConfig` holds
   the tunable params + explicit `enabled` flags (plain data); a separate `RenderResources` bundle
   holds the pipeline/sampler handles. A feature is **active = `config.<feature>.enabled` && its
   resource valid**. Explicit enable flags decouple "configured" from "on".
3. **Config keeps human units** (e.g. `sunAngularRadiusDeg`, `cos()` computed internally).
4. **MSAA honesty.** The config centralizes the *decision* (`aa.msaaSamples`), but MSAA pipelines
   are sample-count-specific — the app still supplies + swaps sample-count-matched mesh+sky
   pipelines when toggling (same shape as HDR needing RGBA16Float pipelines). Documented, not
   abstracted away.
5. **Cluster grid honesty.** The froxel grid moves into `ClusterConfig` for visibility/recording,
   but stays **effectively compile-time** in G1 (froxel buffer sizing + shader loop bounds are
   coupled to it); runtime resize is out of scope. Defaults mirror the `cluster::` constexpr.

## Plan (G1–G3, mirrors physics P1–P3)
- **G1 — un-bury + unify (NO behavior change).**
  - G1a: `engine/graphics/render/graphics_config.h` — `ShadowConfig/SkyConfig/FogConfig/AAConfig/
    ClusterConfig` nested in `GraphicsConfig` (+ `bool hdr`), and a `RenderResources` handle bundle.
    Defaults == current `Impl`/`constexpr` values. Un-buries `shadow::MAP_SIZE` → `shadow.mapSize`
    (now actually tunable) and the froxel grid → `ClusterConfig` (compile-time-effective, decision 5).
  - G1b: `Renderer::Impl` holds a `GraphicsConfig config_` + `RenderResources resources_` instead of
    the scattered fields; add `setConfig()` / `setResources()`; re-express the 7 existing `setX()`
    setters as **thin wrappers** that mutate `config_`/`resources_` (keeps every call site working);
    `render()` reads `config_`/`resources_`. Verify suite 163/0 byte-identical + benchmarks flat.
- **G2 — override layering + presets.** `GraphicsConfigOverride` (all-`std::optional`, nested) +
  `resolve(base, override)` + named presets (`presets::performance()`, `presets::cinematic()`).
  Unit test `graphics.config`.
- **G3 — write-only serialization.** `serialize(GraphicsConfig)→text` + FNV-1a hash + version, for
  screenshot/repro metadata. Reader deferred (same locked call as physics P3).

## Out of scope / later
- Key/value READER (launch-time overrides) — with a tools/CLI consumer.
- Runtime froxel-grid resize (needs buffer + shader coherence).
- A "pipeline variant set" helper to remove the app's MSAA sample-count pipeline juggling (the
  config makes the *data* clean; the pipeline objects remain a GPU reality).
- Per-material / per-object graphics overrides (this config is renderer-global).
