#  Physics configuration system — review + plan (2026-07-04)

Centralize the physics engine's scattered tuning knobs into one overridable, per-sim config with a
run-history audit trail. Review requested after the contact-geometry work surfaced how many knobs exist.

## Problem — knobs are scattered across 3 structs + hardcoded constants
- `physics::WorldDef` (`world.h`): gravity, velocityIterations, substeps, broadphase, linear/angularDamping,
  threadPool, parallelThreshold, continuousDetection.
- `physics_env::EnvConfig` (`environment.h`): articulation, gravity, controlDt, substeps, velocityIterations,
  linear/angularDamping, maxTorque, actionMode, kp, kd, groundPlane, groundFriction, backend.
- `diff::DiffModel` contact + `DiffEnvironment`: contactGround, groundK/C/Beta/Mu, frictionVref,
  contactIntegration, gravity, controlDt, substeps (auto), DiffContact.
- `PhysicsMaterial` (per body): friction, restitution.
- **Hardcoded `constexpr` (not exposed!):** realtime {kBaumgarte 0.2, kSlop 0.005, kMaxCorrection 2,
  kAabbMargin 0.01, kJointBaumgarte 0.2}; reduced {kMaxPerManifold 4, kBeta 0.2, kSlop 0.001, kMaxCorr 4,
  kIters 12, kEps 1e-12}.

Problems: duplication/drift (gravity, substeps, damping in 3 places, hand-translated in each ctor);
~11 solver knobs un-tunable without editing source; no run history/repro; no override layering; no
serialization infra in-tree (checked — no json/toml/yaml dep).

## Locked decisions
1. **Value-type config, NO global mutable singleton.** Preserve the engine's explicit plain-data style
   (parallel VecEnv sims + determinism depend on no hidden shared state). "Centralized" = one struct
   definition + one defaults source + one serializer, not one global instance.
2. **Configs are header/code-defined** (`SimConfig::defaults()` + named configs). History via git.
3. **Serialization: WRITE-only now, READER deferred.** A ~30-line dump of the RESOLVED config → text gives
   the run-history/repro audit. The key/value READER (launch-time overrides without recompiling) waits
   for a training launcher/CLI to consume it — format is defined by the writer.
4. Keep `Real` (physics) vs `double` (diff contact) typed correctly; nest by concern (SolverConfig) not
   per-backend; verify no perf/determinism regression when former-`constexpr` become struct reads; no
   reflection/registry.

## Plan (P1–P3 approved)
- **P1 — unify + un-bury (NO behavior change).**
  - P1a: `engine/physics/config.h` — `SolverConfig` (the un-buried realtime + reduced constants, defaults ==
    current values). Add `SolverConfig solver` to `WorldDef`; backends read `wd.solver.*` instead of the
    `constexpr`. Verify suite + step/reduced benchmarks byte-identical.
    **DONE (2026-07-04):** `config.h`+`SolverConfig` (10 knobs: realtime contactBaumgarte/contactSlop/
    maxCorrection/aabbMargin/jointBaumgarte, reduced pgsIterations/maxContactsPerManifold/reducedBaumgarte/
    reducedSlop/reducedMaxCorrection). `WorldDef::solver` added; realtime reads `def_.solver.*`, reduced
    stores `solver_(def.solver)`. All `constexpr` removed (kept `kEps` numeric guard). Reads hoisted out of
    hot loops (no perf/determinism change). Suite 132/0, ctest 7/7 — behavior identical.
  - P1b: `SimConfig` aggregate {gravity, controlDt, backend, damping, `SolverConfig`, `ContactConfig`
    (diff), `ActuationConfig` {maxTorque, actionMode, kp, kd}, material/ground defaults}. `EnvConfig` and
    `DiffEnvironment` derive their `WorldDef`/`DiffModel` from `SimConfig` (`toWorldDef()` etc.) — one
    source of truth, remove hand-duplication. `SimConfig::defaults()` is the single defaults home.
    **DONE (2026-07-04):** `SimConfig` (flat: gravity, controlDt, substeps, velocityIterations,
    linear/angularDamping, backend, groundPlane, groundFriction, maxTorque, actionMode, kp, kd,
    `SolverConfig solver`) added to `config.h`; `Backend` + `ActionMode` enums MOVED to `config.h`
    (namespace-preserving). `WorldDef toWorldDef(const SimConfig&)` inline in `world.h`. `EnvConfig` is
    now `{ ArticulationDef articulation; SimConfig sim; }` — one knob surface for the RL sim (solver
    internals now reachable/recordable at the env level); `Environment` uses `toWorldDef(config_.sim)`;
    all ~6 call sites remapped (`cfg.substeps`→`cfg.sim.substeps` etc.). Suite 132/0, byte-identical.
    **SCOPING NOTE:** folded WorldDef+EnvConfig into `SimConfig`; **left the DIFF engine's config as
    `DiffModel`** (separate engine — header-only under physics/diff, no physics_env dep; its knobs are
    already one struct). Folding diff contact into the same `SimConfig` would fight module layering +
    large churn for a second, disjoint engine. `SimConfig` = the maximal/reduced RL sim; `DiffModel` =
    the differentiable sim. Revisit if a single cross-engine config is wanted (would move `ContactConfig`
    + `ContactIntegration` into `config.h` and rework `DiffModel` to hold a `ContactConfig`).
- **P2 — override layering.** `SimConfigOverride` (all-`std::optional`) + `resolve(base, override)` (or a
  fluent builder). Per-sim config = base + its override. Named experiment configs in a `configs/` area.
  **DONE (2026-07-04):** `SimConfigOverride` (sparse; each field `std::optional`, incl. nested
  `SolverConfigOverride`) + `resolve(SimConfig base, override)` (`applyOverride` per field) in `config.h`;
  `configs.h` with named presets (`configs::realtime()`, `configs::reducedHumanoid()`). Test
  `tst/physics/unit/config.cpp`: resolve applies set fields / keeps base for unset (incl nested solver),
  threads through `toWorldDef`, preset + per-sim override compose. Suite 135/0.
- **P3 — write-only serialization + history.** Dump resolved `SimConfig` → compact text (home-grown
  key/value; ~30 lines) + a config hash + engine config-version, logged per run. (Reader deferred.)
  **DONE (2026-07-04):** `config_io.h` — `serialize(SimConfig)→string` (key=value, one knob/line, floats
  `%.9g`), `configHash` (FNV-1a 64-bit over the text), `dump()` = serialize + `configHash=<hex>` line,
  `kConfigVersion=1` (schema tag, emitted first line). `backendName`/`actionModeName` helpers. Test
  `config_serialize_and_hash`: expected keys present, hash deterministic + identity-sensitive (one knob
  change ⇒ different hash), dump carries the hash. Suite 136/0. READER still deferred (locked decision).

## Out of scope / later
- **Cross-engine config unification (DEFERRED — user OK'd the two-surface split 2026-07-04, revisit later).**
  Today `SimConfig` = the maximal/reduced RL sim; the differentiable engine keeps its own `DiffModel`
  (contact knobs `groundK/C/beta/mu/frictionVref` + `contactIntegration`). To unify into ONE config:
  move `ContactConfig` + `ContactIntegration` into `config.h` (pure data), rework `DiffModel` to hold a
  `ContactConfig`, and add it to `SimConfig` (or a shared parent). Bounded but touches the diff engine +
  layering; do it when a single cross-engine tuning/serialization surface is actually needed.
- Key/value READER (launch-time overrides / sweeps) — add with the training launcher.
- Per-run config logging wired into VecEnv/training loop (needs the trainer).
- Exposing broadphase/threadpool policy through configs beyond what already exists.
