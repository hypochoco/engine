//
//  configs.h
//  engine::physics::configs
//
//  Named SimConfig presets (P2) — the organized, versioned home for base tuning configs. An
//  experiment/sim picks a preset and layers a sparse `SimConfigOverride` on top via `resolve()`:
//
//      const SimConfig cfg = resolve(configs::reducedHumanoid(), { .substeps = 64, .maxTorque = 50 });
//
//  These cover physics/solver/actuation tuning only; the RL model (ArticulationDef) is composed at
//  the physics_env layer (EnvConfig = { articulation, SimConfig sim }). Keep additions here so every
//  run's base config is discoverable + git-tracked in one place.
//

#pragma once

#include "engine/physics/config.h"

namespace engine::physics::configs {

// Baseline realtime (maximal-coordinate) sim — the engine defaults.
inline SimConfig realtime() { return SimConfig{}; }

// Reduced-coordinate (Featherstone) humanoid RL sim: finer substeps + a lower torque clamp, which
// the reduced ABA needs under strong torque with hard joint limits (mirrors the reduced_env tests).
inline SimConfig reducedHumanoid() {
    SimConfig c;
    c.backend = Backend::Reduced;
    c.substeps = 48;
    c.maxTorque = Real(40);
    return c;
}

} // namespace engine::physics::configs
