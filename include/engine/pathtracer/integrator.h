//
//  integrator.h
//  engine::pathtracer
//
//  The reference CPU path-tracing integrator (step 1). Monte-Carlo estimator of the rendering
//  equation: camera rays, area-light next-event estimation, cosine-weighted indirect bounces,
//  Fresnel dielectrics + perfect mirrors, Russian-roulette termination. Deterministic (per-pixel
//  seeded RNG). Brute-force intersection (BVH deferred). Ported from an old CPU tracer with its
//  direct-lighting / sampling bugs fixed — see the salvage-assessment note.
//

#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "engine/pathtracer/scene.h"

namespace engine::pt {

struct Settings {
    int      samplesPerPixel    = 16;
    int      maxDepth           = 8;      // hard recursion cap (safety net around Russian roulette)
    float    continuationProb   = 0.8f;   // Russian-roulette survival prob for indirect diffuse bounces
    bool     directLightingOnly = false;  // skip indirect (direct + emission only)
    uint32_t seed               = 1u;     // base RNG seed — same seed ⇒ identical image (determinism)
};

// Render `scene` at width×height. Returns linear HDR radiance per pixel, row-major (size w*h),
// top-to-bottom / left-to-right. Deterministic for a fixed Settings::seed.
std::vector<glm::vec3> render(const Scene& scene, uint32_t width, uint32_t height,
                              const Settings& settings);

// Reinhard tone map (L/(1+L)) + sRGB-ish gamma → 8-bit RGBA, row-major (size w*h*4). Convenience
// for writing/inspecting an image; the engine's own tonemap.slang is used on the GPU path later.
std::vector<uint8_t> toneMap(const std::vector<glm::vec3>& hdr, float exposure = 1.0f);

} // namespace engine::pt
