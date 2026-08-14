#include "harness/harness.h"
//
//  profile.cpp
//  engine::tst — core / unit
//
//  Tests the scoped-zone profiler API (engine::prof): sample recording, windowed stats +
//  percentile ordering, frame markers driving fps, ScopedZone RAII, and format(). The API is
//  compiled unconditionally (independent of the ENGINE_PROFILING macro), so this test runs in any
//  build config; it exercises record()/report() directly rather than the compile-gated macros.
//

#include <cstdio>
#include <thread>

#include "engine/core/profile/profile.h"

using namespace engine;

TST_CASE(core, unit, profile_zone_stats) {
    prof::reset();

    // Record a known spread of samples into one zone.
    for (int i = 1; i <= 100; ++i) prof::record("work", static_cast<double>(i));  // 1..100 ms

    prof::Report rep = prof::report();
    TST_REQUIRE_MSG(!rep.zones.empty(), "expected at least one zone");

    const prof::ZoneStats* z = nullptr;
    for (const auto& zs : rep.zones) if (zs.name == "work") z = &zs;
    TST_REQUIRE_MSG(z != nullptr, "zone 'work' should be present");

    // Only the last kWindow samples are retained for the windowed stats.
    const double expectedCalls = 100;
    TST_REQUIRE_MSG(z->calls == static_cast<uint64_t>(expectedCalls), "calls should count every record");

    // Windowed spread (last min(100,kWindow) samples). Percentiles must be ordered and bounded.
    TST_REQUIRE_MSG(z->minMs <= z->avgMs && z->avgMs <= z->maxMs, "min <= avg <= max");
    TST_REQUIRE_MSG(z->p50Ms <= z->p95Ms && z->p95Ms <= z->p99Ms, "p50 <= p95 <= p99");
    TST_REQUIRE_MSG(z->p99Ms <= z->maxMs + 1e-9, "p99 <= max");
    TST_REQUIRE_MSG(z->lastMs == expectedCalls, "last sample recorded");

    std::printf("profile zone: calls=%llu avg=%.2f p95=%.2f p99=%.2f max=%.2f\n",
                static_cast<unsigned long long>(z->calls), z->avgMs, z->p95Ms, z->p99Ms, z->maxMs);
}

TST_CASE(core, unit, profile_frame_markers) {
    prof::reset();

    // A few frame markers with a small sleep → a positive frame time and fps.
    for (int i = 0; i < 5; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        prof::endFrame();
    }
    prof::Report rep = prof::report();
    TST_REQUIRE_MSG(rep.frame == 5, "frame count should track endFrame() calls");
    TST_REQUIRE_MSG(rep.cpuFrameMs > 0.0, "frame time should be positive");
    TST_REQUIRE_MSG(rep.fps > 0.0, "fps should be positive");
    std::printf("profile frame: frames=%llu cpuFrameMs=%.3f fps=%.1f\n",
                static_cast<unsigned long long>(rep.frame), rep.cpuFrameMs, rep.fps);
}

TST_CASE(core, unit, profile_scoped_zone) {
    prof::reset();
    {
        prof::ScopedZone z("scoped");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    prof::Report rep = prof::report();
    bool found = false;
    for (const auto& zs : rep.zones)
        if (zs.name == "scoped") { found = true; TST_REQUIRE_MSG(zs.lastMs > 0.0, "scoped zone timed > 0"); }
    TST_REQUIRE_MSG(found, "ScopedZone should record on destruction");

    // format() produces a non-empty human-readable summary.
    const std::string text = prof::format(rep);
    TST_REQUIRE_MSG(!text.empty(), "format() should produce output");
    std::printf("%s", text.c_str());
}
