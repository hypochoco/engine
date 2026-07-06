//
//  config.cpp
//  engine::tst / graphics / unit
//
//  The centralized graphics config (G1–G3): defaults match the former hardcoded values; the layered
//  `GraphicsConfigOverride` + `resolve(base, override)` (sparse overrides replace only their set
//  fields, everything else keeps the base); named presets (`presets::`); and write-only serialization
//  + config hash + version. Header-only logic — no GPU device needed.
//

#include <cstdio>
#include <string>

#include "engine/graphics/render/graphics_config.h"
#include "engine/graphics/render/graphics_config_io.h"
#include "harness/harness.h"

using namespace engine::render;

namespace {
bool contains(const std::string& hay, const char* needle) { return hay.find(needle) != std::string::npos; }
}

TST_CASE(graphics, unit, config_defaults) {
    const GraphicsConfig d;
    // Everything off by default (parity baseline).
    TST_REQUIRE(!d.hdr && !d.shadow.enabled && !d.sky.enabled && !d.fog.enabled && !d.cluster.enabled);
    TST_REQUIRE(d.aa.msaaSamples == 1 && !d.aa.fxaa);
    // Defaults equal the former hardcoded values.
    TST_APPROX(d.shadow.orthoHalfExtent, 25.0f, 1e-6);
    TST_APPROX(d.shadow.depthRange, 100.0f, 1e-6);
    TST_APPROX(d.shadow.bias, 0.0018f, 1e-9);
    TST_REQUIRE(d.shadow.mapSize == 2048);
    TST_APPROX(d.sky.sunAngularRadiusDeg, 1.0f, 1e-6);
    TST_APPROX(d.sky.glowExponent, 250.0f, 1e-6);
    TST_APPROX(d.fog.inscatterExponent, 8.0f, 1e-6);
    TST_REQUIRE(d.cluster.gridX == 12 && d.cluster.gridY == 12 && d.cluster.gridZ == 24 && d.cluster.maxPerCluster == 64);
}

TST_CASE(graphics, unit, config_resolve_override) {
    const GraphicsConfig base;
    GraphicsConfigOverride ov;
    ov.hdr = true;
    ov.aa.msaaSamples = 4;
    ov.shadow.enabled = true;
    ov.shadow.mapSize = 4096;
    ov.fog.density = 0.02f;     // nested override

    const GraphicsConfig r = resolve(base, ov);
    // overridden fields take the override value...
    TST_REQUIRE(r.hdr && r.aa.msaaSamples == 4 && r.shadow.enabled && r.shadow.mapSize == 4096);
    TST_APPROX(r.fog.density, 0.02f, 1e-9);
    // ...everything unset keeps the base
    TST_APPROX(r.shadow.orthoHalfExtent, base.shadow.orthoHalfExtent, 1e-6);
    TST_APPROX(r.shadow.bias, base.shadow.bias, 1e-9);
    TST_REQUIRE(r.aa.fxaa == base.aa.fxaa);
    TST_REQUIRE(!r.fog.enabled);   // density set, but enabled was not overridden
    TST_APPROX(r.sky.glowExponent, base.sky.glowExponent, 1e-6);
    std::printf("config_resolve: hdr=%d msaa=%u shadowMap=%u fogDensity=%.3f (base extent kept=%.1f)\n",
                r.hdr, r.aa.msaaSamples, r.shadow.mapSize, r.fog.density, r.shadow.orthoHalfExtent);
}

TST_CASE(graphics, unit, config_presets) {
    const GraphicsConfig base = presets::baseline();
    TST_REQUIRE(base.aa.msaaSamples == 1 && !base.aa.fxaa && !base.hdr);

    const GraphicsConfig perf = presets::performance();
    TST_REQUIRE(perf.aa.fxaa && perf.aa.msaaSamples == 1 && perf.shadow.mapSize == 1024);

    const GraphicsConfig cine = presets::cinematic();
    TST_REQUIRE(cine.hdr && cine.aa.msaaSamples == 4 && cine.shadow.mapSize == 4096);
    TST_REQUIRE(cine.sky.enabled && cine.fog.enabled);

    // preset + a per-scene override
    GraphicsConfigOverride o; o.aa.msaaSamples = 8;
    const GraphicsConfig tuned = resolve(presets::cinematic(), o);
    TST_REQUIRE(tuned.aa.msaaSamples == 8 && tuned.hdr && tuned.shadow.mapSize == 4096);   // rest kept from preset
}

TST_CASE(graphics, unit, config_serialize_and_hash) {
    const GraphicsConfig base;
    const std::string s = serialize(base);
    TST_REQUIRE(contains(s, "graphicsConfigVersion=1"));
    TST_REQUIRE(contains(s, "aa.msaaSamples=1"));
    TST_REQUIRE(contains(s, "shadow.mapSize=2048"));
    TST_REQUIRE(contains(s, "fog.inscatterExponent=8"));
    TST_REQUIRE(contains(s, "cluster.gridZ=24"));

    // hash is deterministic + identity-sensitive
    TST_REQUIRE(configHash(base) == configHash(GraphicsConfig{}));
    GraphicsConfigOverride ov; ov.aa.msaaSamples = 4;
    const GraphicsConfig tuned = resolve(base, ov);
    TST_REQUIRE(configHash(tuned) != configHash(base));
    TST_REQUIRE(contains(serialize(tuned), "aa.msaaSamples=4"));

    // dump() carries the hash line
    const std::string d = dump(presets::cinematic());
    TST_REQUIRE(contains(d, "hdr=1") && contains(d, "aa.msaaSamples=4") && contains(d, "configHash="));
    std::printf("config_serialize: base hash=%016llx cinematic dump %zu bytes\n",
                static_cast<unsigned long long>(configHash(base)), d.size());
}
