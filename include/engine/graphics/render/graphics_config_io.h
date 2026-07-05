//
//  graphics_config_io.h
//  engine::graphics / render
//
//  Write-only GraphicsConfig serialization (G3) — the screenshot/repro audit trail. Dumps a resolved
//  GraphicsConfig to compact, human-readable, diffable `key=value` text (one knob per line), plus a
//  stable `configHash` (identity tag) and an engine `graphicsConfigVersion` (schema tag so old logs
//  stay interpretable). Log `dump(cfg)` alongside a screenshot / capture. Mirrors physics config_io.
//
//  READER is intentionally omitted (see notes/investigations/2026-07-05-graphics-config-system.md):
//  add key/value parsing when a tools/CLI layer needs launch-time overrides from files. Resources
//  (GPU handles) are not serialized — this captures the tunable data + toggles only.
//

#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "engine/graphics/render/graphics_config.h"

namespace engine::render {

// Bump when the GraphicsConfig schema changes (fields added/removed/renamed).
inline constexpr int kGraphicsConfigVersion = 1;

namespace detail {
inline void kv(std::string& s, const char* key, double v) { char b[96]; std::snprintf(b, sizeof b, "%s=%.9g\n", key, v); s += b; }
inline void kv(std::string& s, const char* key, int v)    { char b[96]; std::snprintf(b, sizeof b, "%s=%d\n", key, v); s += b; }
inline void kv3(std::string& s, const char* key, const glm::vec3& v) {
    char b[128]; std::snprintf(b, sizeof b, "%s=%.9g,%.9g,%.9g\n", key, v.x, v.y, v.z); s += b;
}
}

// Resolved GraphicsConfig → compact key=value text (does NOT include the hash line; see dump()).
inline std::string serialize(const GraphicsConfig& c) {
    using detail::kv; using detail::kv3;
    std::string s;
    kv(s, "graphicsConfigVersion", kGraphicsConfigVersion);
    kv(s, "hdr", c.hdr ? 1 : 0);

    kv(s, "shadow.enabled", c.shadow.enabled ? 1 : 0);
    kv(s, "shadow.orthoHalfExtent", c.shadow.orthoHalfExtent);
    kv(s, "shadow.depthRange", c.shadow.depthRange);
    kv(s, "shadow.bias", c.shadow.bias);
    kv(s, "shadow.mapSize", static_cast<int>(c.shadow.mapSize));

    kv(s, "sky.enabled", c.sky.enabled ? 1 : 0);
    kv3(s, "sky.zenith", c.sky.zenith);
    kv3(s, "sky.horizon", c.sky.horizon);
    kv3(s, "sky.ground", c.sky.ground);
    kv3(s, "sky.sunColor", c.sky.sunColor);
    kv(s, "sky.sunAngularRadiusDeg", c.sky.sunAngularRadiusDeg);
    kv(s, "sky.glowExponent", c.sky.glowExponent);
    kv(s, "sky.glowStrength", c.sky.glowStrength);
    kv(s, "sky.brightness", c.sky.brightness);

    kv(s, "fog.enabled", c.fog.enabled ? 1 : 0);
    kv(s, "fog.density", c.fog.density);
    kv(s, "fog.heightFalloff", c.fog.heightFalloff);
    kv(s, "fog.baseHeight", c.fog.baseHeight);
    kv(s, "fog.inscatterExponent", c.fog.inscatterExponent);
    kv3(s, "fog.color", c.fog.color);
    kv3(s, "fog.inscatterColor", c.fog.inscatterColor);

    kv(s, "aa.msaaSamples", static_cast<int>(c.aa.msaaSamples));
    kv(s, "aa.fxaa", c.aa.fxaa ? 1 : 0);

    kv(s, "cluster.enabled", c.cluster.enabled ? 1 : 0);
    kv(s, "cluster.gridX", static_cast<int>(c.cluster.gridX));
    kv(s, "cluster.gridY", static_cast<int>(c.cluster.gridY));
    kv(s, "cluster.gridZ", static_cast<int>(c.cluster.gridZ));
    kv(s, "cluster.maxPerCluster", static_cast<int>(c.cluster.maxPerCluster));
    return s;
}

// FNV-1a 64-bit over the serialized text — a stable identity tag for a config.
inline uint64_t configHash(const GraphicsConfig& c) {
    const std::string s = serialize(c);
    uint64_t h = 1469598103934665603ull;
    for (const char ch : s) { h ^= static_cast<uint8_t>(ch); h *= 1099511628211ull; }
    return h;
}

// serialize() + a trailing `configHash=<hex>` line — the blob to log per capture.
inline std::string dump(const GraphicsConfig& c) {
    std::string s = serialize(c);
    char b[48]; std::snprintf(b, sizeof b, "configHash=%016llx\n", static_cast<unsigned long long>(configHash(c)));
    s += b;
    return s;
}

} // namespace engine::render
