//
//  config.h
//  engine::physics
//
//  Centralized, tunable physics configuration (P1 of the config-system plan; see
//  notes/investigations/2026-07-04-physics-config-system.md). This header un-buries the solver
//  tuning constants that used to be file-scope `constexpr` inside the backends, so they can be
//  tuned/swept/recorded rather than requiring a source edit + recompile. Plain-data + value-type
//  (no global singleton) — passed explicitly via `WorldDef`, preserving determinism + parallel envs.
//
//  Defaults here MUST equal the previous hardcoded values (P1 is a no-behavior-change refactor).
//

#pragma once

#include <optional>

#include "engine/physics/types.h"

namespace engine::physics {

// Solver tuning knobs, formerly hardcoded `constexpr` in the backend .cpp files.
struct SolverConfig {
    // --- Realtime (maximal-coordinate sequential-impulse) backend ---
    Real contactBaumgarte = Real(0.2);      // contact position-correction fraction
    Real contactSlop      = Real(0.005);    // allowed penetration before correction (m)
    Real maxCorrection    = Real(2);        // cap on Baumgarte correction velocity (m/s)
    Real aabbMargin       = Real(0.01);     // broadphase AABB fattening (m)
    Real jointBaumgarte   = Real(0.2);      // joint position-drift correction fraction (no slop)

    // --- Reduced (Featherstone/ABA + PGS contact) backend ---
    int  pgsIterations          = 12;       // PGS sweeps for the reduced contact solve
    int  maxContactsPerManifold = 4;        // manifold reduction cap (deepest N per (link,plane))
    Real reducedBaumgarte       = Real(0.2);   // reduced-contact position-correction fraction
    Real reducedSlop            = Real(0.001); // allowed penetration before correction (m)
    Real reducedMaxCorrection   = Real(4);     // cap on reduced Baumgarte correction velocity (m/s)
};

// Which physics backend a sim uses (moved here from world.h so SimConfig can reference it).
enum class Backend {
    Realtime,   // maximal-coordinate sequential-impulse solver (contacts + joint constraints)
    Reduced,    // reduced-coordinate Featherstone/ABA articulation (Phase E)
    Cuda,       // batched reduced + smoothed-contact ABA on the GPU (NVIDIA only, requires ENGINE_CUDA).
                // Selected via the CudaVecEnv path — NOT createPhysicsWorld() (it is not a PhysicsWorld).
                // The device kernel runs the SAME templated diff ABA as the CPU Reduced/diff path.
};

// How an RL action vector is interpreted (see physics_env). Torque = raw joint torques;
// PDTarget = desired joint position tracked by a PD servo.
enum class ActionMode { Torque, PDTarget };

// The centralized, tunable configuration for one simulation — the single place that lists every
// knob (P1b). Plain-data + value-type: pass by value, override per sim, serialize for history.
// `WorldDef` is derived from this via toWorldDef() (world.h); `physics_env::EnvConfig` embeds it.
struct SimConfig {
    // --- integration / world ---
    Vec3 gravity{ 0, Real(-9.81), 0 };
    Real controlDt = Real(1) / Real(60);   // one control step (env-level)
    int  substeps = 8;                      // physics substeps per control step
    int  velocityIterations = 16;
    Real linearDamping = Real(0.05);
    Real angularDamping = Real(0.1);
    Backend backend = Backend::Realtime;

    // --- ground plane ---
    bool groundPlane = true;
    Real groundFriction = Real(0.9);

    // --- actuation (RL action write-path) ---
    Real maxTorque = Real(150);             // per-DOF torque clamp
    ActionMode actionMode = ActionMode::Torque;
    Real kp = Real(150);                    // PD position gain (PDTarget mode)
    Real kd = Real(15);                     // PD velocity gain (PDTarget mode)

    // --- solver internals ---
    SolverConfig solver{};
};

// ---- Override layering (P2) ------------------------------------------------------------------
// A sparse override: only the set (engaged optional) fields replace the base. Lets a per-sim /
// per-experiment config specify just the knobs it changes on top of a base (defaults or a preset).
template <class T> void applyOverride(T& dst, const std::optional<T>& o) { if (o) dst = *o; }

struct SolverConfigOverride {
    std::optional<Real> contactBaumgarte, contactSlop, maxCorrection, aabbMargin, jointBaumgarte;
    std::optional<int>  pgsIterations, maxContactsPerManifold;
    std::optional<Real> reducedBaumgarte, reducedSlop, reducedMaxCorrection;
};

struct SimConfigOverride {
    std::optional<Vec3>       gravity;
    std::optional<Real>       controlDt;
    std::optional<int>        substeps, velocityIterations;
    std::optional<Real>       linearDamping, angularDamping;
    std::optional<Backend>    backend;
    std::optional<bool>       groundPlane;
    std::optional<Real>       groundFriction;
    std::optional<Real>       maxTorque;
    std::optional<ActionMode> actionMode;
    std::optional<Real>       kp, kd;
    SolverConfigOverride      solver;
};

// Resolve an override onto a base, returning the effective SimConfig (base for every unset field).
inline SimConfig resolve(SimConfig base, const SimConfigOverride& o) {
    applyOverride(base.gravity, o.gravity);
    applyOverride(base.controlDt, o.controlDt);
    applyOverride(base.substeps, o.substeps);
    applyOverride(base.velocityIterations, o.velocityIterations);
    applyOverride(base.linearDamping, o.linearDamping);
    applyOverride(base.angularDamping, o.angularDamping);
    applyOverride(base.backend, o.backend);
    applyOverride(base.groundPlane, o.groundPlane);
    applyOverride(base.groundFriction, o.groundFriction);
    applyOverride(base.maxTorque, o.maxTorque);
    applyOverride(base.actionMode, o.actionMode);
    applyOverride(base.kp, o.kp);
    applyOverride(base.kd, o.kd);
    applyOverride(base.solver.contactBaumgarte, o.solver.contactBaumgarte);
    applyOverride(base.solver.contactSlop, o.solver.contactSlop);
    applyOverride(base.solver.maxCorrection, o.solver.maxCorrection);
    applyOverride(base.solver.aabbMargin, o.solver.aabbMargin);
    applyOverride(base.solver.jointBaumgarte, o.solver.jointBaumgarte);
    applyOverride(base.solver.pgsIterations, o.solver.pgsIterations);
    applyOverride(base.solver.maxContactsPerManifold, o.solver.maxContactsPerManifold);
    applyOverride(base.solver.reducedBaumgarte, o.solver.reducedBaumgarte);
    applyOverride(base.solver.reducedSlop, o.solver.reducedSlop);
    applyOverride(base.solver.reducedMaxCorrection, o.solver.reducedMaxCorrection);
    return base;
}

} // namespace engine::physics
