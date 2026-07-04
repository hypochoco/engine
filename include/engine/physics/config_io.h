//
//  config_io.h
//  engine::physics
//
//  Write-only SimConfig serialization (P3) — the run-history / reproducibility audit trail. Dumps a
//  resolved SimConfig to compact, human-readable, diffable `key=value` text (one knob per line),
//  plus a stable `configHash` (identity tag for a run) and an engine `configVersion` (schema tag so
//  old logs stay interpretable). A downstream trainer logs `dump(cfg)` alongside each run's results.
//
//  READER is intentionally omitted (see notes/investigations/2026-07-04-physics-config-system.md):
//  add key/value parsing when a training launcher needs launch-time overrides from files.
//

#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "engine/physics/config.h"

namespace engine::physics {

// Bump when the SimConfig schema changes (fields added/removed/renamed) so serialized logs are
// interpretable across engine versions.
inline constexpr int kConfigVersion = 1;

inline const char* backendName(Backend b) { return b == Backend::Reduced ? "Reduced" : "Realtime"; }
inline const char* actionModeName(ActionMode m) { return m == ActionMode::PDTarget ? "PDTarget" : "Torque"; }

namespace detail {
inline void kv(std::string& s, const char* key, double v) { char b[80]; std::snprintf(b, sizeof b, "%s=%.9g\n", key, v); s += b; }
inline void kv(std::string& s, const char* key, int v)    { char b[80]; std::snprintf(b, sizeof b, "%s=%d\n", key, v); s += b; }
inline void kv(std::string& s, const char* key, const char* v) { s += key; s += '='; s += v; s += '\n'; }
}

// Resolved SimConfig → compact key=value text (does NOT include the hash line; see dump()).
inline std::string serialize(const SimConfig& c) {
    using detail::kv;
    std::string s;
    kv(s, "configVersion", kConfigVersion);
    kv(s, "gravity.x", c.gravity.x); kv(s, "gravity.y", c.gravity.y); kv(s, "gravity.z", c.gravity.z);
    kv(s, "controlDt", c.controlDt);
    kv(s, "substeps", c.substeps);
    kv(s, "velocityIterations", c.velocityIterations);
    kv(s, "linearDamping", c.linearDamping);
    kv(s, "angularDamping", c.angularDamping);
    kv(s, "backend", backendName(c.backend));
    kv(s, "groundPlane", c.groundPlane ? 1 : 0);
    kv(s, "groundFriction", c.groundFriction);
    kv(s, "maxTorque", c.maxTorque);
    kv(s, "actionMode", actionModeName(c.actionMode));
    kv(s, "kp", c.kp);
    kv(s, "kd", c.kd);
    kv(s, "solver.contactBaumgarte", c.solver.contactBaumgarte);
    kv(s, "solver.contactSlop", c.solver.contactSlop);
    kv(s, "solver.maxCorrection", c.solver.maxCorrection);
    kv(s, "solver.aabbMargin", c.solver.aabbMargin);
    kv(s, "solver.jointBaumgarte", c.solver.jointBaumgarte);
    kv(s, "solver.pgsIterations", c.solver.pgsIterations);
    kv(s, "solver.maxContactsPerManifold", c.solver.maxContactsPerManifold);
    kv(s, "solver.reducedBaumgarte", c.solver.reducedBaumgarte);
    kv(s, "solver.reducedSlop", c.solver.reducedSlop);
    kv(s, "solver.reducedMaxCorrection", c.solver.reducedMaxCorrection);
    return s;
}

// FNV-1a 64-bit over the serialized text — a stable identity tag for a run's config.
inline uint64_t configHash(const SimConfig& c) {
    const std::string s = serialize(c);
    uint64_t h = 1469598103934665603ull;
    for (const char ch : s) { h ^= static_cast<uint8_t>(ch); h *= 1099511628211ull; }
    return h;
}

// serialize() + a trailing `configHash=<hex>` line — the blob to log per run.
inline std::string dump(const SimConfig& c) {
    std::string s = serialize(c);
    char b[48]; std::snprintf(b, sizeof b, "configHash=%016llx\n", static_cast<unsigned long long>(configHash(c)));
    s += b;
    return s;
}

} // namespace engine::physics
