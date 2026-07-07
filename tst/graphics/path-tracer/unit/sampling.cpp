//
//  sampling.cpp
//  engine::tst / graphics / path-tracer / unit
//
//  Unit tests for the pure path-tracer primitives (engine/pathtracer/sampling.h): Fresnel-Schlick,
//  cosine-weighted hemisphere sampling (validity + pdf + a Monte-Carlo integral with a known answer),
//  uniform triangle sampling (in-triangle + converges to the centroid), and the orthonormal basis.
//

#include <cmath>
#include <cstdio>
#include <random>

#include <glm/glm.hpp>

#include "engine/pathtracer/sampling.h"
#include "harness/harness.h"

using namespace engine::pt;

TST_CASE(pathtracer, unit, fresnel_schlick) {
    // Normal incidence (cosI = 1) → R0 = ((1-1.5)/(1+1.5))² = 0.04.
    TST_APPROX(fresnelSchlick(1.0f, 1.0f, 1.5f), 0.04f, 1e-4);
    // Grazing (cosI = 0) → total reflectance 1.
    TST_APPROX(fresnelSchlick(0.0f, 1.0f, 1.5f), 1.0f, 1e-4);
    // Swapping etaI/etaT leaves R0 unchanged (it is squared).
    TST_APPROX(fresnelSchlick(1.0f, 1.5f, 1.0f), 0.04f, 1e-4);
    // Monotonic: reflectance rises as the angle grazes (cosI decreases).
    TST_REQUIRE(fresnelSchlick(1.0f, 1.0f, 1.5f) < fresnelSchlick(0.5f, 1.0f, 1.5f));
    TST_REQUIRE(fresnelSchlick(0.5f, 1.0f, 1.5f) < fresnelSchlick(0.1f, 1.0f, 1.5f));
}

TST_CASE(pathtracer, unit, orthonormal_basis) {
    const glm::vec3 normals[] = {
        {0,1,0}, {0,-1,0}, {1,0,0}, {0,0,1}, glm::normalize(glm::vec3(1,2,3)), glm::normalize(glm::vec3(-3,0.1f,0.2f))
    };
    for (glm::vec3 n : normals) {
        glm::vec3 t, b; orthonormalBasis(n, t, b);
        TST_APPROX(glm::length(t), 1.0f, 1e-4);
        TST_APPROX(glm::length(b), 1.0f, 1e-4);
        TST_APPROX(glm::dot(t, n), 0.0f, 1e-4);
        TST_APPROX(glm::dot(b, n), 0.0f, 1e-4);
        TST_APPROX(glm::dot(t, b), 0.0f, 1e-4);
    }
}

TST_CASE(pathtracer, unit, cosine_hemisphere) {
    std::mt19937 gen(12345);
    std::uniform_real_distribution<float> U(0.0f, 1.0f);
    const glm::vec3 n = glm::normalize(glm::vec3(0.3f, 0.8f, -0.2f));

    // Per-sample validity: in the hemisphere, unit length, pdf == cosθ/π.
    for (int i = 0; i < 2000; ++i) {
        float pdf = 0.0f;
        const glm::vec3 wi = cosineSampleHemisphere(n, U(gen), U(gen), pdf);
        const float c = glm::dot(wi, n);
        TST_REQUIRE_MSG(c >= -1e-4f, "sample must lie in the hemisphere");
        TST_APPROX(glm::length(wi), 1.0f, 1e-3);
        TST_APPROX(pdf, std::max(c, 0.0f) / PI_F, 1e-3);
    }

    // Monte-Carlo estimate of ∫_hemisphere cos²θ dω = 2π/3 ≈ 2.0944 (bounded integrand ⇒ low variance).
    double acc = 0.0; const int N = 400000;
    for (int i = 0; i < N; ++i) {
        float pdf = 0.0f;
        const glm::vec3 wi = cosineSampleHemisphere(n, U(gen), U(gen), pdf);
        const float c = glm::max(glm::dot(wi, n), 0.0f);
        if (pdf > 0.0f) acc += (c * c) / pdf;   // f/pdf, f = cos²θ
    }
    const double est = acc / N;
    std::printf("cosine hemisphere ∫cos²θ dω est=%.4f (expect %.4f)\n", est, 2.0 * M_PI / 3.0);
    TST_REQUIRE_MSG(std::fabs(est - 2.0 * M_PI / 3.0) < 0.02, "cosine-pdf integral must match 2π/3");
}

TST_CASE(pathtracer, unit, triangle_sampling) {
    const glm::vec3 v0(0, 0, 0), v1(1, 0, 0), v2(0, 1, 0);   // right triangle in z=0, centroid (1/3,1/3,0)
    std::mt19937 gen(999);
    std::uniform_real_distribution<float> U(0.0f, 1.0f);

    glm::dvec3 mean(0.0);
    const int N = 200000;
    for (int i = 0; i < N; ++i) {
        const glm::vec3 p = sampleTriangleUniform(v0, v1, v2, U(gen), U(gen));
        // Inside the triangle: x,y ≥ 0, x+y ≤ 1, z == 0.
        TST_REQUIRE_MSG(p.x >= -1e-4f && p.y >= -1e-4f && p.x + p.y <= 1.0f + 1e-4f, "sample must be inside the triangle");
        TST_APPROX(p.z, 0.0f, 1e-5);
        mean += glm::dvec3(p);
    }
    mean /= double(N);
    // Uniform sampling ⇒ expected value is the centroid.
    TST_REQUIRE_MSG(std::fabs(mean.x - 1.0 / 3.0) < 5e-3 && std::fabs(mean.y - 1.0 / 3.0) < 5e-3,
                    "uniform triangle samples must average to the centroid");
    std::printf("triangle sampling mean=(%.4f,%.4f) (expect 0.3333,0.3333)\n", mean.x, mean.y);
}
