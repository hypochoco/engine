//
//  config.cpp
//  engine::tst / physics / unit
//
//  P2: the layered config system — `SimConfigOverride` + `resolve(base, override)` (sparse overrides
//  replace only their set fields, everything else keeps the base), named presets (`configs::`), and
//  derivation into the backend `WorldDef` (`toWorldDef`). Also confirms defaults are unchanged.
//

#include <cstdio>
#include <string>

#include "engine/physics/config.h"
#include "engine/physics/config_io.h"
#include "engine/physics/configs.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine::physics;

namespace {
bool contains(const std::string& hay, const char* needle) { return hay.find(needle) != std::string::npos; }
}

TST_CASE(physics, unit, config_defaults_and_solver) {
    const SimConfig d;
    TST_REQUIRE(d.substeps == 8 && d.velocityIterations == 16);
    TST_REQUIRE(d.backend == Backend::Realtime);
    TST_REQUIRE(d.solver.pgsIterations == 12 && d.solver.maxContactsPerManifold == 4);
    TST_APPROX(d.solver.reducedMaxCorrection, 4.0, 1e-9);
    TST_APPROX(d.maxTorque, 150.0, 1e-9);
}

TST_CASE(physics, unit, config_resolve_override) {
    const SimConfig base;   // defaults
    SimConfigOverride ov;
    ov.substeps = 64;
    ov.backend = Backend::Reduced;
    ov.maxTorque = Real(40);
    ov.solver.pgsIterations = 20;   // nested solver override

    const SimConfig r = resolve(base, ov);
    // overridden fields take the override value...
    TST_REQUIRE(r.substeps == 64 && r.backend == Backend::Reduced);
    TST_APPROX(r.maxTorque, 40.0, 1e-9);
    TST_REQUIRE(r.solver.pgsIterations == 20);
    // ...everything unset keeps the base
    TST_REQUIRE(r.velocityIterations == base.velocityIterations);
    TST_APPROX(r.kp, base.kp, 1e-9);
    TST_REQUIRE(r.solver.maxContactsPerManifold == base.solver.maxContactsPerManifold);
    TST_APPROX(r.controlDt, base.controlDt, 1e-9);

    // the resolved config threads through to the backend WorldDef
    const WorldDef wd = toWorldDef(r);
    TST_REQUIRE(wd.substeps == 64 && wd.solver.pgsIterations == 20);
    std::printf("config_resolve: substeps=%d backend=%d pgsIters=%d maxTorque=%.0f (base velIters kept=%d)\n",
                r.substeps, static_cast<int>(r.backend), r.solver.pgsIterations, static_cast<double>(r.maxTorque), r.velocityIterations);
}

TST_CASE(physics, unit, config_presets) {
    const SimConfig rt = configs::realtime();
    TST_REQUIRE(rt.backend == Backend::Realtime && rt.substeps == 8);

    const SimConfig red = configs::reducedHumanoid();
    TST_REQUIRE(red.backend == Backend::Reduced && red.substeps == 48);
    TST_APPROX(red.maxTorque, 40.0, 1e-9);

    // preset + a per-sim override
    const SimConfig tuned = resolve(configs::reducedHumanoid(), [] { SimConfigOverride o; o.substeps = 64; return o; }());
    TST_REQUIRE(tuned.backend == Backend::Reduced && tuned.substeps == 64);
    TST_APPROX(tuned.maxTorque, 40.0, 1e-9);   // kept from the preset
}

// P3: write-only serialization + config hash + version (the run-history audit trail).
TST_CASE(physics, unit, config_serialize_and_hash) {
    const SimConfig base;
    const std::string s = serialize(base);
    TST_REQUIRE(contains(s, "configVersion=1"));
    TST_REQUIRE(contains(s, "substeps=8"));
    TST_REQUIRE(contains(s, "backend=Realtime"));
    TST_REQUIRE(contains(s, "actionMode=Torque"));
    TST_REQUIRE(contains(s, "solver.pgsIterations=12"));
    TST_REQUIRE(contains(s, "solver.reducedMaxCorrection=4"));

    // hash is deterministic + identity-sensitive
    TST_REQUIRE(configHash(base) == configHash(SimConfig{}));      // same values → same hash
    SimConfigOverride ov; ov.substeps = 64;
    const SimConfig tuned = resolve(base, ov);
    TST_REQUIRE(configHash(tuned) != configHash(base));            // one knob differs → different hash
    TST_REQUIRE(contains(serialize(tuned), "substeps=64"));

    // dump() carries the hash line for per-run logging
    const std::string d = dump(configs::reducedHumanoid());
    TST_REQUIRE(contains(d, "backend=Reduced") && contains(d, "substeps=48") && contains(d, "configHash="));
    std::printf("config_serialize: base hash=%016llx tuned hash=%016llx  (dump %zu bytes)\n",
                static_cast<unsigned long long>(configHash(base)), static_cast<unsigned long long>(configHash(tuned)), d.size());
}
