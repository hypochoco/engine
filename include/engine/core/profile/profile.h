//
//  profile.h
//  engine::core / profile
//
//  A tiny, general-purpose CPU profiler for named scoped zones. Records per-zone timing samples
//  into a windowed accumulator and reports mean / min / max / percentiles (p50/p95/p99) plus a
//  frame-time / fps series driven by frame markers. Dependency-free (std only) so it is available
//  in every build configuration.
//
//  COMPILE-GATED: instrumentation is only emitted when ENGINE_PROFILING is defined (set by the
//  CMake option of the same name, auto-off in Release). When it is NOT defined, the
//  ENGINE_PROFILE_SCOPE / ENGINE_PROFILE_FRAME macros expand to nothing (zero overhead) and
//  prof::enabled() is a constexpr false — so consumers can compile out their reporting with
//  `if constexpr (engine::prof::enabled())`.
//
//  Usage:
//      void frame() {
//          { ENGINE_PROFILE_SCOPE("extract"); scene::extract(...); }
//          { ENGINE_PROFILE_SCOPE("render");  renderer.render(...); }
//          ENGINE_PROFILE_FRAME();                  // once per frame
//      }
//      if constexpr (engine::prof::enabled())       // compiled out in Release
//          std::puts(engine::prof::format(engine::prof::report()).c_str());
//

#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace engine::prof {

// Windowed statistics for one named zone (all times in milliseconds).
struct ZoneStats {
    std::string name;
    uint64_t    calls = 0;   // total calls since the last reset()
    double      lastMs = 0.0;
    double      avgMs  = 0.0; // mean over the recent window
    double      minMs  = 0.0; // min over the recent window
    double      maxMs  = 0.0; // max over the recent window
    double      p50Ms  = 0.0;
    double      p95Ms  = 0.0;
    double      p99Ms  = 0.0;
};

// A snapshot of the whole profiler state.
struct Report {
    double                 cpuFrameMs = 0.0; // mean wall time between frame markers (window)
    double                 fps        = 0.0; // 1000 / cpuFrameMs
    uint64_t               frame      = 0;   // frame markers since the last reset()
    std::vector<ZoneStats> zones;            // sorted by avgMs descending
};

// True only when built with profiling instrumentation. constexpr so reporting code can be
// compiled out entirely with `if constexpr (engine::prof::enabled())`.
constexpr bool enabled() {
#if defined(ENGINE_PROFILING)
    return true;
#else
    return false;
#endif
}

// Record one timing sample (milliseconds) for a named zone. Thread-safe. Prefer ENGINE_PROFILE_SCOPE.
void record(std::string_view name, double milliseconds);

// Mark a frame boundary: samples wall-clock since the previous marker into the frame-time series.
// Prefer ENGINE_PROFILE_FRAME. No-op-equivalent cost when profiling is disabled (never called by
// the macro in that case).
void endFrame();

// Snapshot the current windowed statistics. Cheap enough to call ~once per second for logging.
Report report();

// Clear all accumulated statistics and the frame series.
void reset();

// Number of most-recent samples each zone keeps for the windowed stats / percentiles.
constexpr std::size_t kWindow = 128;

// A compact multi-line summary (one line per zone: avg / p95 / p99 / last / calls), plus a header
// line with fps and mean CPU frame time. For console or an on-screen HUD.
std::string format(const Report&);

// RAII scope timer: on destruction, records the elapsed wall time into `name`. `name` must outlive
// the object (string literals are ideal). Only instantiated by ENGINE_PROFILE_SCOPE when enabled.
class ScopedZone {
public:
    explicit ScopedZone(std::string_view name)
        : name_(name), start_(std::chrono::steady_clock::now()) {}
    ~ScopedZone() {
        const auto end = std::chrono::steady_clock::now();
        record(name_, std::chrono::duration<double, std::milli>(end - start_).count());
    }
    ScopedZone(const ScopedZone&) = delete;
    ScopedZone& operator=(const ScopedZone&) = delete;

private:
    std::string_view                                   name_;
    std::chrono::steady_clock::time_point              start_;
};

} // namespace engine::prof

// --- Instrumentation macros (the compile-out point) --------------------------------------------
#define ENGINE_PROF_CONCAT_(a, b) a##b
#define ENGINE_PROF_CONCAT(a, b) ENGINE_PROF_CONCAT_(a, b)

#if defined(ENGINE_PROFILING)
    // Times the enclosing scope, recording into zone `name` (a string literal or other view that
    // outlives the scope).
    #define ENGINE_PROFILE_SCOPE(name) \
        ::engine::prof::ScopedZone ENGINE_PROF_CONCAT(engine_prof_zone_, __LINE__){name}
    // Marks a frame boundary (call once per frame).
    #define ENGINE_PROFILE_FRAME() ::engine::prof::endFrame()
#else
    #define ENGINE_PROFILE_SCOPE(name) ((void)0)
    #define ENGINE_PROFILE_FRAME()     ((void)0)
#endif
