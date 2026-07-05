# Atmosphere — aerial perspective + height fog (RF6 atmosphere)

Point-in-time investigation. Settles the design for atmospheric fog on geometry, the layer after
the procedural sky (`2026-07-05-sky-atmosphere.md`). Folds into `core/` when done.

## Goal & guidance

The sky colors the *background*; atmosphere colors the *geometry* so distant objects fade into
that sky (aerial perspective) instead of standing in sharp relief. Guidance (2026-07-05, carried
from the sky work): realtime graphical feature — **lean toward performance**, physically-inspired
not accurate. Test early/often; **benchmark before AND after**. All open questions answered
"recommended": aerial-perspective + height fog (not a physically-based single-scattering sky),
height fog included in v1, **off by default** (opt-in like sky/shadows/tonemap).

Not a LUT pipeline (see the sky note + the LUT discussion): this is the cheap analytic stand-in for
Hillaire's 3D aerial-perspective froxel LUT — same visual goal, a fraction of the cost. The LUT
upgrade stays a clean additive future track behind the same `setFog` opt-in.

## Decision 1 — where it runs: the forward fragment shader (`mesh.slang`)

Fog on geometry needs the fragment's **world position** + the **lit color**. A separate fullscreen
fog pass would have to read the depth buffer, but our depth is transient/memoryless (the same
constraint that put the sky in the forward pass). So fog is the **last step of the mesh fragment
shader**: compute lit color as today, then blend toward the fog color. The sky pass is unchanged
(the background already *is* the sky; fog only touches geometry, and geometry at max distance fogs
to ~the sky color, so the two meet seamlessly).

## Decision 2 — fog factor: distance × height, exponential

```
dist   = length(worldPos - camPos)
// exponential distance extinction
// height term: fog is dense low, thins with altitude (grounds the scene)
heightK = exp(-max(worldPos.y - fogBaseY, 0) * fogHeightFalloff)
optical = dist * fogDensity * heightK
fogAmt  = 1 - exp(-optical)          // 0 near, →1 far / low
finalColor = lerp(litColor, fogColor, saturate(fogAmt))
```

Cheap (one `exp`, one `length`, a few muls). The height term uses the *fragment's* height (a
per-fragment approximation of the proper along-ray density integral — good enough, and standard for
cheap height fog). `fogBaseY` + `fogHeightFalloff` place and shape the fog layer.

## Decision 3 — fog color: sky-consistent base + sun in-scatter (the "aerial" part)

Flat gray fog looks like a curtain. Aerial perspective = two terms:
- **Base extinction tint** (`fogColor`): an atmosphere color near the sky's horizon color, so
  geometry fading out meets the sky seamlessly.
- **Sun in-scatter** (Mie-like): looking toward the sun through haze brightens/warms —
  `inscatter = pow(saturate(dot(viewDir, sunDir)), fogInscatterG) * fogInscatterColor`, added into
  the fog color weighted by `fogAmt`. This is what sells distant haze glowing near the sun.

```
viewDir = normalize(worldPos - camPos)
mu      = dot(viewDir, sunDir)                 // sunDir = toward the sun (same as the light)
inscat  = pow(saturate(mu), fogInscatterG) * fogInscatterColor
fogCol  = fogColor + inscat                     // scene-referred (HDR); survives tonemap
finalColor = lerp(litColor, fogCol, fogAmt)
```

Coupled to the same `light.direction` that drives the sun/sky/shadows. HDR/scene-referred so the
in-scatter can exceed 1 and tonemap handles it.

## Decision 4 — API + data path

- `Renderer::setFog(density, heightFalloff, baseHeight, color, inscatterColor, inscatterG)` opt-in;
  a `setFog()` / invalid/zero-density disables. **Off by default** ⇒ existing pixel-parity anchors
  (`graphics.mesh`/`scene`/`triangle`) unchanged.
- Params ride in the `mesh.slang` **Globals** constant buffer (extend it + its CPU mirror
  `GlobalUniforms`). Need: camera world pos (for `dist`/`viewDir`), fog color, in-scatter color,
  and `(density, heightFalloff, baseY, inscatterG)` packed. Camera pos = `inverse(view)[3]`.
  A fog-enable flag reuses `params` or a new packed field. Layout must stay 16-byte aligned and the
  CPU mirror must match exactly (learned from the shadow/sky work).

## Decision 5 — testability (`graphics.fog`, headless)

- **Distance washout**: two identical boxes, near vs far. Fog on ⇒ the far box's color is pulled
  toward the fog color (its albedo/contrast washed out) while the near box keeps its albedo.
- **Height fog**: a tall box ⇒ its base (low) is foggier than its top (high).
- **Sun in-scatter**: with fog on, a distant fogged region *toward* the sun is brighter/warmer than
  one *away* from the sun.
- **No-regression / parity**: fog OFF ⇒ pixels identical to today (assert the parity anchor pixel).

## Decision 6 — benchmark before/after

`render_graph` fog on/off row (reuse the instance sweep). A few fragment ALU ⇒ expect ~free;
the point is to *measure* it, same as the sky (~0.01 ms). Record next to the RF6 entry.

## Deferred (later, per the plan)

- Hillaire **aerial-perspective 3D LUT** (accurate, froxel volume) behind the same `setFog`.
- **Volumetric** fog / light shafts (god-rays) — needs shadow-map sampling along the ray.
- Fog influencing the **sky** pass; physically-based single-scattering sky.
- Fog as a function of a real **participating-media** density field (clouds).
