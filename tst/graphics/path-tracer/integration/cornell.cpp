//
//  cornell.cpp
//  engine::tst / graphics / path-tracer / integration
//
//  Correctness invariants for the reference CPU path tracer (engine::pt), on a classic Cornell box
//  (white floor/ceiling/back, red left wall, green right wall, ceiling area light). Not pixel-exact
//  vs a ground-truth render — checks the physically-meaningful properties that the fixed integrator
//  must satisfy: finite output, a lit scene, red/green colour bleed on the correct sides, indirect
//  light adds energy over direct-only, and determinism for a fixed seed.
//

#include <cmath>
#include <cstdio>
#include <vector>

#include <glm/glm.hpp>

#include "engine/pathtracer/integrator.h"
#include "engine/pathtracer/scene.h"
#include "harness/harness.h"

using namespace engine;

namespace {

void addQuad(pt::Scene& s, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, uint32_t mat) {
    const glm::vec3 fn = glm::normalize(glm::cross(b - a, c - a));
    auto tri = [&](glm::vec3 p0, glm::vec3 p1, glm::vec3 p2) {
        pt::Triangle t; t.v0 = p0; t.v1 = p1; t.v2 = p2; t.n0 = t.n1 = t.n2 = fn; t.material = mat;
        s.triangles.push_back(t);
    };
    tri(a, b, c);
    tri(a, c, d);
}

// Cornell box in the cube [-1,1]^3, open front at z=+1, camera looking down -z from z>1.
pt::Scene cornellBox() {
    pt::Scene s;
    const uint32_t white = s.addMaterial({ .albedo = glm::vec3(0.75f) });
    const uint32_t red   = s.addMaterial({ .albedo = glm::vec3(0.75f, 0.10f, 0.10f) });
    const uint32_t green = s.addMaterial({ .albedo = glm::vec3(0.10f, 0.75f, 0.10f) });
    pt::Material lightMat; lightMat.albedo = glm::vec3(0.0f); lightMat.emission = glm::vec3(18.0f);
    const uint32_t light = s.addMaterial(lightMat);

    addQuad(s, {-1,-1,-1}, { 1,-1,-1}, { 1,-1, 1}, {-1,-1, 1}, white);  // floor
    addQuad(s, {-1, 1,-1}, {-1, 1, 1}, { 1, 1, 1}, { 1, 1,-1}, white);  // ceiling
    addQuad(s, {-1,-1,-1}, {-1, 1,-1}, { 1, 1,-1}, { 1,-1,-1}, white);  // back
    addQuad(s, {-1,-1,-1}, {-1,-1, 1}, {-1, 1, 1}, {-1, 1,-1}, red);    // left  (red)
    addQuad(s, { 1,-1,-1}, { 1, 1,-1}, { 1, 1, 1}, { 1,-1, 1}, green);  // right (green)

    // Ceiling area light (just below the ceiling, facing down).
    const float e = 0.30f, y = 0.999f;
    addQuad(s, {-e, y, -e}, {-e, y, e}, { e, y, e}, { e, y, -e}, light);

    s.camera.eye     = glm::vec3(0.0f, 0.0f, 3.2f);
    s.camera.forward = glm::vec3(0.0f, 0.0f, -1.0f);
    s.camera.up      = glm::vec3(0.0f, 1.0f, 0.0f);
    s.camera.fovY    = glm::radians(50.0f);
    s.finalize();
    return s;
}

float lum(const glm::vec3& c) { return glm::dot(c, glm::vec3(0.2126f, 0.7152f, 0.0722f)); }

glm::vec3 meanOverColumns(const std::vector<glm::vec3>& img, uint32_t w, uint32_t h, uint32_t x0, uint32_t x1) {
    glm::vec3 acc(0.0f); uint32_t n = 0;
    for (uint32_t y = 0; y < h; ++y)
        for (uint32_t x = x0; x < x1; ++x) { acc += img[y * w + x]; ++n; }
    return n ? acc / static_cast<float>(n) : acc;
}

} // namespace

TST_CASE(pathtracer, integration, cornell_box) {
    const uint32_t W = 48, H = 48;
    const pt::Scene scene = cornellBox();
    TST_REQUIRE_MSG(scene.emissive.size() == 2, "cornell box should have 2 emissive triangles (the light quad)");

    pt::Settings full; full.samplesPerPixel = 64; full.maxDepth = 6; full.continuationProb = 0.8f; full.seed = 7;
    const std::vector<glm::vec3> img = pt::render(scene, W, H, full);

    // 1. finite, non-negative output — no NaNs/Infs from the sampling/estimator math.
    bool finite = true;
    for (const glm::vec3& c : img)
        if (!std::isfinite(c.r) || !std::isfinite(c.g) || !std::isfinite(c.b) || c.r < 0 || c.g < 0 || c.b < 0) { finite = false; break; }
    TST_REQUIRE_MSG(finite, "path tracer output must be finite and non-negative");

    // 2. the scene is actually lit (mean luminance well above black).
    double mean = 0.0; for (const glm::vec3& c : img) mean += lum(c);
    mean /= (W * H);
    TST_REQUIRE_MSG(mean > 0.02, "scene should be meaningfully lit");

    // 3. colour bleed: left third reads red, right third reads green (the coloured side walls).
    const glm::vec3 left  = meanOverColumns(img, W, H, 0, W / 3);
    const glm::vec3 right = meanOverColumns(img, W, H, 2 * W / 3, W);
    TST_REQUIRE_MSG(left.r  > left.g  && left.r  > left.b,  "left side should be reddish (red wall)");
    TST_REQUIRE_MSG(right.g > right.r && right.g > right.b, "right side should be greenish (green wall)");

    // 4. indirect light adds energy: full render is brighter than direct-lighting-only.
    pt::Settings direct = full; direct.directLightingOnly = true;
    const std::vector<glm::vec3> imgD = pt::render(scene, W, H, direct);
    double meanD = 0.0; for (const glm::vec3& c : imgD) meanD += lum(c);
    meanD /= (W * H);
    TST_REQUIRE_MSG(mean > meanD, "full (direct+indirect) should be brighter than direct-only");

    // 5. determinism: same seed ⇒ identical image.
    const std::vector<glm::vec3> img2 = pt::render(scene, W, H, full);
    bool identical = (img.size() == img2.size());
    for (size_t i = 0; identical && i < img.size(); ++i) identical = (img[i] == img2[i]);
    TST_REQUIRE_MSG(identical, "same seed must produce an identical image (determinism)");

    std::printf("cornell: mean L full=%.4f direct=%.4f | left(r=%.3f g=%.3f) right(r=%.3f g=%.3f)\n",
                mean, meanD, left.r, left.g, right.r, right.g);
    std::printf("path tracer cornell_box ok\n");
}
