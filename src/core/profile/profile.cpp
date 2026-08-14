//
//  profile.cpp
//  engine::core / profile
//
//  Implementation of the scoped-zone CPU profiler. The registry is a process-global keyed by zone
//  name; each zone keeps a ring buffer of the last kWindow samples for windowed mean/min/max and
//  percentiles. A separate ring holds inter-frame wall times for the frame-time / fps series.
//
//  The functions are defined unconditionally (so they always link), but they are only *exercised*
//  when ENGINE_PROFILING is defined — the macros in profile.h are the compile-out point, so in a
//  non-profiling build record()/endFrame() are simply never called and cost nothing.
//

#include "engine/core/profile/profile.h"

#include <algorithm>
#include <cstdio>
#include <mutex>
#include <unordered_map>

namespace engine::prof {
namespace {

// A fixed-capacity ring of the most recent samples (milliseconds) plus lifetime counters.
struct Series {
    double   ring[kWindow] = {};
    std::size_t count = 0;   // number of valid samples (<= kWindow)
    std::size_t head  = 0;   // next write index
    uint64_t calls = 0;      // total pushes since reset
    double   last  = 0.0;

    void push(double ms) {
        ring[head] = ms;
        head = (head + 1) % kWindow;
        if (count < kWindow) ++count;
        ++calls;
        last = ms;
    }

    // Fill windowed stats into (avg,min,max,p50,p95,p99). Copies the window and uses nth_element.
    void summarize(double& avg, double& mn, double& mx,
                   double& p50, double& p95, double& p99) const {
        avg = mn = mx = p50 = p95 = p99 = 0.0;
        if (count == 0) return;
        double tmp[kWindow];
        double sum = 0.0;
        for (std::size_t i = 0; i < count; ++i) { tmp[i] = ring[i]; sum += ring[i]; }
        avg = sum / static_cast<double>(count);
        const auto pct = [&](double q) {
            std::size_t k = static_cast<std::size_t>(q * (count - 1) + 0.5);
            if (k >= count) k = count - 1;
            std::nth_element(tmp, tmp + k, tmp + count);
            return tmp[k];
        };
        // min/max are the 0 and 1 quantiles; compute percentiles first (they reorder tmp, fine).
        p50 = pct(0.50);
        p95 = pct(0.95);
        p99 = pct(0.99);
        mn = *std::min_element(tmp, tmp + count);
        mx = *std::max_element(tmp, tmp + count);
    }
};

struct Registry {
    std::mutex                             mutex;
    std::unordered_map<std::string, Series> zones;
    Series                                 frame;   // inter-frame wall times
    uint64_t                               frameCount = 0;
    std::chrono::steady_clock::time_point  lastFrame{};
    bool                                   haveLastFrame = false;
};

Registry& registry() {
    static Registry r;
    return r;
}

} // namespace

void record(std::string_view name, double milliseconds) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.zones[std::string(name)].push(milliseconds);
}

void endFrame() {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const auto now = std::chrono::steady_clock::now();
    if (r.haveLastFrame)
        r.frame.push(std::chrono::duration<double, std::milli>(now - r.lastFrame).count());
    r.lastFrame     = now;
    r.haveLastFrame = true;
    ++r.frameCount;
}

Report report() {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);

    Report rep;
    rep.frame = r.frameCount;
    {
        double avg, mn, mx, p50, p95, p99;
        r.frame.summarize(avg, mn, mx, p50, p95, p99);
        rep.cpuFrameMs = avg;
        rep.fps        = avg > 0.0 ? 1000.0 / avg : 0.0;
    }
    rep.zones.reserve(r.zones.size());
    for (const auto& [name, series] : r.zones) {
        ZoneStats z;
        z.name  = name;
        z.calls = series.calls;
        z.lastMs = series.last;
        series.summarize(z.avgMs, z.minMs, z.maxMs, z.p50Ms, z.p95Ms, z.p99Ms);
        rep.zones.push_back(std::move(z));
    }
    std::sort(rep.zones.begin(), rep.zones.end(),
              [](const ZoneStats& a, const ZoneStats& b) { return a.avgMs > b.avgMs; });
    return rep;
}

void reset() {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.zones.clear();
    r.frame = Series{};
    r.frameCount = 0;
    r.haveLastFrame = false;
}

std::string format(const Report& rep) {
    std::string out;
    char line[256];
    std::snprintf(line, sizeof(line),
                  "[perf] frame %llu  %.2f ms  (%.0f fps)\n",
                  static_cast<unsigned long long>(rep.frame), rep.cpuFrameMs, rep.fps);
    out += line;
    for (const auto& z : rep.zones) {
        std::snprintf(line, sizeof(line),
                      "  %-16s avg %6.3f  p95 %6.3f  p99 %6.3f  max %6.3f ms\n",
                      z.name.c_str(), z.avgMs, z.p95Ms, z.p99Ms, z.maxMs);
        out += line;
    }
    return out;
}

} // namespace engine::prof
