# Sky / atmosphere — design (RF6 sky)

Point-in-time investigation. Settles the design for the sky/atmosphere pass in the clustered
forward+ render framework, then folds the outcome into `core/`. Sequenced right after shadows
(see `2026-07-04-render-framework-plan.md` §4a graph, §10 step 6).

## Goal & guidance

Replace the flat clear color with a believable, **sun-coupled** sky. Guidance (2026-07-05):
this is a *graphical* feature for **realtime**, so **lean toward performance** over physical
accuracy. Test early/often; **benchmark before AND after** to understand the cost.

So: *not* a full atmospheric-scattering LUT chain. A cheap analytic sky that looks good and
costs ~fill-rate + a handful of ALU ops per background pixel.

## Decision 1 — approach: cheap analytic gradient + sun, not Preetham/Hosek/Hillaire

Options considered:
- **Preetham / Hosek-Wilkie** analytic daylight models — physically-grounded, one evaluation
  per pixel, but a chunk of constants/coefficients and a turbidity fit; more than we need and
  heavier per-pixel.
- **Hillaire (2020) multi-scattering LUTs** — the modern realtime-accurate choice, but it's a
  transmittance LUT + multi-scatter LUT + sky-view LUT pipeline (several compute passes). Great
  target *later*; overkill now and against the "lean performance" call.
- **Hand-tuned analytic gradient + sun disc/glow** ← **chosen for v1.** A zenith→horizon color
  gradient with a physically-*inspired* (not accurate) tint, plus a sun disc and a forward-scatter
  glow (Mie-ish), all driven by the sun direction. No textures, no LUTs, ~20 ALU ops/pixel.

The parameterization is intentionally physically-*inspired* so it reads correctly as the sun
moves (day → sunset): horizon warms and reddens near the sun, zenith stays blue and darkens, the
whole sky dims as the sun sets. But every term is a cheap closed form, not an integral.

### v1 sky model (per view ray `d`, sun dir `s` = toward the sun)

```
mu       = dot(d, s)                       // view·sun alignment
up       = clamp(d.y, 0, 1)                // 0 at horizon, 1 at zenith
sunAmt   = clamp(s.y, 0, 1)                // day/night factor from sun elevation

// gradient: horizon color -> zenith color by a shaped 'up' (pow for a tighter horizon band)
grad     = mix(horizonColor, zenithColor, pow(up, 0.5)) * sunAmt-driven brightness
// horizon warms toward the sun's azimuth
grad    += horizonSunTint * (mu*0.5+0.5)^k

// sun disc (hard-ish) + glow (broad Mie-like forward scatter)
disc     = smoothstep(cosSunSize - e, cosSunSize, mu) * sunIntensity   // scene-referred (HDR, >1)
glow     = pow(max(mu,0), glowExp) * glowStrength
sky      = grad + (disc + glow) * sunColor

// below the horizon (d.y < 0): fade to a dim ground/haze color (no true ground plane needed)
```

All colors/scalars come from a small constant buffer so the app (and later a day-night driver)
can tune them; sensible defaults live in the renderer. HDR output (sun disc can be >1) → tonemap
resolves it. No physical units — knobs, not radiometry.

## Decision 2 — pass mechanics: fullscreen triangle, depth-tested against opaque

- **Fullscreen triangle** (3 verts synthesized from `SV_VertexID`, covers NDC) at **z = far
  (ndc.z = 1)**. No vertex buffer.
- **Depth test = LessEqual, depth write = OFF**, sharing the **opaque pass's depth buffer**.
  Opaque fragments wrote depth < 1; the sky triangle at z=1 fails `1 <= storedOpaque` there, so it
  **only fills background** pixels (depth still == cleared far). Zero overdraw of the scene; O(1)
  extra geometry. (This is why the sky runs *after* opaque, not before — cheaper than shading sky
  behind everything.)
- **View-ray reconstruction**: in the vertex/fragment, take the pixel's NDC `(x, y, 1)`, transform
  by `invViewProj`, divide by w → world point on the far plane; `dir = normalize(worldFar -
  cameraPos)`. Pass `invViewProj` + `cameraPos` via the sky constant buffer. (Reuses the camera the
  opaque pass already has.)
- **Target**: writes the **HDR** color target (RGBA16F) when tonemapping is on, else the final LDR
  target directly. Runs **before tonemap**, after opaque (and before transparent, when that
  exists).

RHI note: this needs the graphics pipeline to actually honor **depth-test-without-write** and the
**LessEqual** compare + a depth attachment that is loaded (not cleared) by the sky pass. The
depth-only shadow work already exercised depth pipelines; confirm the raster state path supports
`depthWrite=false, depthCompare=LessEqual` and a load (don't-clear) depth attachment in the graph.

## Decision 3 — sun coupling

The sky's sun is the **same** `RenderView.light.direction` that drives the directional light and
shadows. `s = normalize(-light.direction)` (toward the sun). This keeps sky, sun disc, key light,
and shadows consistent for free as the sun rotates (the `shadow_scene` demo already animates it).
Sky sun color/intensity are separate knobs (the disc is much brighter than the key light term).

## Decision 4 — height fog: deferred to a follow-up

The framework plan mentions optional height fog. Keeping v1 minimal (lean performance): ship the
sky first, add exponential height fog as a **separate** small pass/term afterward (it needs the
opaque depth to fog *geometry*, which is a different input than the background-only sky). Noted,
not now.

## Decision 5 — testability (headless pixel test)

`graphics.sky` (headless, offscreen HDR→readback or LDR):
- **Gradient**: a pixel looking up (zenith) is bluer/darker than a pixel looking at the horizon.
- **Sun disc**: looking straight at the sun yields a much brighter pixel than looking away.
- **Depth correctness (the important invariant)**: render a scene with an opaque object; the
  object's pixels are **identical** with the sky pass on vs off (sky only fills background), while
  background pixels change from the clear color to the sky. This proves the depth test.
- **Sun coupling**: move the sun; the bright region in the image follows the sun direction.

## Benchmark plan

`graphics.render_graph` **before** (current: shadows/HDR path) and **after** adding the sky pass,
same scenes. Expectation: a fullscreen depth-tested pass is ~background-fill-rate bound and cheap;
the point is to *measure* the per-frame delta (CPU record should be ~flat; GPU adds one fullscreen
pass over background pixels only). Record numbers in todo.md next to the RF6 entry.

## Deferred (later, per the plan / owner)

- Hillaire multi-scatter LUT sky (physically-accurate realtime) as an upgrade path behind the same
  `setSky` opt-in.
- Aerial perspective / height fog on geometry.
- Sky as an IBL source (diffuse irradiance + specular prefilter) for ambient — currently ambient
  is a flat term.
- Night sky (stars / moon), clouds.
