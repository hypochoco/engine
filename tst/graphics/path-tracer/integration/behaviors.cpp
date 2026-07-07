//
//  behaviors.cpp
//  engine::tst / graphics / path-tracer / integration
//
//  Targeted behavioural tests that isolate individual transport features (vs the whole-Cornell-box
//  smoke test):
//    • no emitter ⇒ a fully black image (no spurious energy),
//    • emitter-triangulation invariance ⇒ splitting the light into more triangles must not change
//      the result (directly guards the fixed area-measure direct-lighting estimator),
//    • a mirror floor reflects the ceiling light (specular path), diffuse floor does not.
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
    tri(a, b, c); tri(a, c, d);
}

// Add a quad emitter as either 2 triangles (fan from a corner) or 4 (fan from the centre) — same
// total area + emission, different triangulation.
void addLight(pt::Scene& s, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, uint32_t mat, int tris) {
    const glm::vec3 fn = glm::normalize(glm::cross(b - a, c - a));
    auto tri = [&](glm::vec3 p0, glm::vec3 p1, glm::vec3 p2) {
        pt::Triangle t; t.v0 = p0; t.v1 = p1; t.v2 = p2; t.n0 = t.n1 = t.n2 = fn; t.material = mat;
        s.triangles.push_back(t);
    };
    if (tris == 4) {
        const glm::vec3 m = 0.25f * (a + b + c + d);
        tri(a, b, m); tri(b, c, m); tri(c, d, m); tri(d, a, m);
    } else {
        tri(a, b, c); tri(a, c, d);
    }
}

// A Cornell-ish box in [-1,1]^3. `mirrorFloor` swaps the floor to a mirror; `lightTris` triangulates
// the ceiling light; `emit` toggles the light emission.
pt::Scene room(bool mirrorFloor, int lightTris, bool emit) {
    pt::Scene s;
    const uint32_t white = s.addMaterial({ .albedo = glm::vec3(0.75f) });
    pt::Material floorMat; floorMat.albedo = glm::vec3(0.9f);
    if (mirrorFloor) floorMat.type = pt::MaterialType::Mirror;
    const uint32_t floor = s.addMaterial(floorMat);
    pt::Material lightMat; lightMat.albedo = glm::vec3(0.0f); lightMat.emission = emit ? glm::vec3(18.0f) : glm::vec3(0.0f);
    const uint32_t light = s.addMaterial(lightMat);

    addQuad(s, {-1,-1,-1}, { 1,-1,-1}, { 1,-1, 1}, {-1,-1, 1}, floor);   // floor
    addQuad(s, {-1, 1,-1}, {-1, 1, 1}, { 1, 1, 1}, { 1, 1,-1}, white);   // ceiling
    addQuad(s, {-1,-1,-1}, {-1, 1,-1}, { 1, 1,-1}, { 1,-1,-1}, white);   // back
    addQuad(s, {-1,-1,-1}, {-1,-1, 1}, {-1, 1, 1}, {-1, 1,-1}, white);   // left
    addQuad(s, { 1,-1,-1}, { 1, 1,-1}, { 1, 1, 1}, { 1,-1, 1}, white);   // right

    const float e = 0.30f, y = 0.999f;
    addLight(s, {-e, y, -e}, {-e, y, e}, { e, y, e}, { e, y, -e}, light, lightTris);

    s.camera.eye = glm::vec3(0, 0, 3.2f);
    s.camera.forward = glm::vec3(0, 0, -1);
    s.camera.up = glm::vec3(0, 1, 0);
    s.camera.fovY = glm::radians(50.0f);
    s.finalize();
    return s;
}

float lum(const glm::vec3& c) { return glm::dot(c, glm::vec3(0.2126f, 0.7152f, 0.0722f)); }
double meanLum(const std::vector<glm::vec3>& img) {
    double m = 0.0; for (const glm::vec3& c : img) m += lum(c); return m / std::max<size_t>(1, img.size());
}
double meanLumRows(const std::vector<glm::vec3>& img, uint32_t w, uint32_t h, uint32_t y0, uint32_t y1) {
    double m = 0.0; uint32_t n = 0;
    for (uint32_t y = y0; y < y1; ++y) for (uint32_t x = 0; x < w; ++x) { m += lum(img[y * w + x]); ++n; }
    return n ? m / n : 0.0;
}

} // namespace

TST_CASE(pathtracer, integration, no_emitter_is_black) {
    const pt::Scene s = room(/*mirrorFloor=*/false, /*lightTris=*/2, /*emit=*/false);
    TST_REQUIRE_MSG(s.emissive.empty(), "scene has no emitters");
    pt::Settings set; set.samplesPerPixel = 8; set.maxDepth = 4; set.seed = 3;
    const std::vector<glm::vec3> img = pt::render(s, 32, 32, set);
    float maxL = 0.0f; for (const glm::vec3& c : img) maxL = std::max(maxL, lum(c));
    TST_REQUIRE_MSG(maxL < 1e-5f, "with no emitter the image must be black (no spurious energy)");
    std::printf("no-emitter maxL=%.2e\n", maxL);
}

TST_CASE(pathtracer, integration, emitter_triangulation_invariance) {
    // Direct-lighting only isolates the NEE estimator. A correct area-measure estimator is invariant
    // to how the emitter is triangulated; the old buggy estimator was not.
    pt::Settings set; set.samplesPerPixel = 128; set.maxDepth = 2; set.directLightingOnly = true; set.seed = 11;
    const std::vector<glm::vec3> img2 = pt::render(room(false, 2, true), 40, 40, set);
    const std::vector<glm::vec3> img4 = pt::render(room(false, 4, true), 40, 40, set);
    const double m2 = meanLum(img2), m4 = meanLum(img4);
    const double rel = std::fabs(m2 - m4) / std::max(1e-6, m2);
    std::printf("triangulation invariance: mean(2 tris)=%.4f mean(4 tris)=%.4f rel=%.3f\n", m2, m4, rel);
    TST_REQUIRE_MSG(rel < 0.06, "direct lighting must be invariant to emitter triangulation");
}

TST_CASE(pathtracer, integration, mirror_reflects_light) {
    pt::Settings set; set.samplesPerPixel = 64; set.maxDepth = 6; set.seed = 5;
    const uint32_t W = 48, H = 48;
    const double diffuseFloor = meanLumRows(pt::render(room(false, 2, true), W, H, set), W, H, 2 * H / 3, H);
    const double mirrorFloor  = meanLumRows(pt::render(room(true,  2, true), W, H, set), W, H, 2 * H / 3, H);
    std::printf("floor region mean L: diffuse=%.4f mirror=%.4f\n", diffuseFloor, mirrorFloor);
    // The mirror floor reflects the bright ceiling light into the camera → substantially brighter.
    TST_REQUIRE_MSG(mirrorFloor > diffuseFloor * 1.3, "mirror floor should reflect the light (brighter than diffuse)");
}
