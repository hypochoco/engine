//
//  integrator.cpp
//  engine::pathtracer
//
//  Reference CPU path tracer. Ported from an old student tracer (path-hypochoco) with its bugs
//  fixed (see notes/investigations/path-tracing/2026-07-06-path-tracer-salvage-assessment.md):
//    • direct lighting uses a correct area-measure estimator (per-sample area/pdf weighting),
//    • uniform triangle sampling (√r1 barycentrics),
//    • cosine-weighted hemisphere sampling (pdf = cosθ/π) instead of biased rejection,
//    • epsilon-offset secondary ray origins (no shadow acne / light leaks),
//    • per-pixel deterministic RNG (no shared rand()).
//  Brute-force intersection (no BVH yet). Two-sided area lights (robust to emitter winding).
//

#include "engine/pathtracer/integrator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include "engine/pathtracer/sampling.h"

namespace engine::pt {
namespace {

constexpr float EPS = 1e-4f;   // ray-origin offset + t_min

struct Rng {
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist{0.0f, 1.0f};
    explicit Rng(uint32_t seed) : gen(seed) {}
    float operator()() { return dist(gen); }
};

// Non-delta BSDF value f(wo, wi). Both point away from the surface. (Mirror/Dielectric are delta —
// handled directly in trace(), not here.)
glm::vec3 brdfEval(const Material& m, const glm::vec3& n, const glm::vec3& wo, const glm::vec3& wi) {
    if (m.type == MaterialType::Glossy) {
        const glm::vec3 r = glm::reflect(-wo, n);   // mirror direction of the view vector
        const float s = std::pow(std::max(glm::dot(r, wi), 0.0f), m.shininess);
        return m.albedo * ((m.shininess + 2.0f) / (2.0f * PI_F)) * s;
    }
    return m.albedo / PI_F;   // Lambertian
}

struct Tracer {
    const Scene&    scene;
    const Settings& s;

    // Next-event estimation against the (two-sided) area lights. Correct area-measure estimator:
    // pick a light uniformly (pdf 1/N), sample it uniformly by area (pdf 1/A), so the 1/pdf weight
    // is A*N; the geometry term G = cosSurf·cosLight / dist² carries the area→solid-angle change.
    glm::vec3 directLight(const glm::vec3& p, const glm::vec3& n, const glm::vec3& wo,
                          const Material& m, Rng& rng) const {
        const size_t N = scene.emissive.size();
        if (N == 0) return glm::vec3(0.0f);

        const uint32_t li = scene.emissive[std::min<size_t>(size_t(rng() * N), N - 1)];
        const Triangle& lt = scene.triangles[li];

        // Uniform point on the light triangle (√r1 barycentrics — fixes the old centroid-biased scheme).
        const glm::vec3 q = sampleTriangleUniform(lt.v0, lt.v1, lt.v2, rng(), rng());

        glm::vec3 d = q - p;
        const float dist2 = glm::dot(d, d);
        if (dist2 < 1e-9f) return glm::vec3(0.0f);
        const float dist = std::sqrt(dist2);
        d /= dist;

        const float cosSurf = glm::dot(n, d);
        glm::vec3 ln = lt.geoNormal();
        float cosLight = glm::dot(ln, -d);
        if (cosLight < 0.0f) cosLight = -cosLight;   // two-sided emitter (robust to winding)
        if (cosSurf <= 0.0f || cosLight <= 0.0f) return glm::vec3(0.0f);

        if (scene.occluded(p + n * EPS, d, dist)) return glm::vec3(0.0f);

        const glm::vec3 f  = brdfEval(m, n, wo, d);
        const glm::vec3 Le = scene.materials[lt.material].emission;
        const float G = (cosSurf * cosLight) / dist2;
        return f * Le * G * lt.area() * static_cast<float>(N);
    }

    glm::vec3 trace(const glm::vec3& o, const glm::vec3& din, int depth, Rng& rng, bool countEmitted) const {
        const glm::vec3 d = glm::normalize(din);
        const Hit h = scene.intersect(o, d, EPS);
        if (!h.valid) return glm::vec3(0.0f);

        const Triangle& tri = scene.triangles[h.tri];
        const Material&  m   = scene.materials[tri.material];
        const glm::vec3  wo  = -d;
        glm::vec3 L(0.0f);

        // --- perfect mirror ---
        if (m.type == MaterialType::Mirror) {
            if (countEmitted && m.emissive()) L += m.emission;
            if (depth + 1 > s.maxDepth) return L;
            const glm::vec3 nn = (glm::dot(h.n, wo) < 0.0f) ? -h.n : h.n;
            const glm::vec3 r  = glm::reflect(d, nn);
            L += m.albedo * trace(h.p + nn * EPS, r, depth + 1, rng, true);
            return L;
        }

        // --- Fresnel dielectric (reflect or refract, Schlick) ---
        if (m.type == MaterialType::Dielectric) {
            if (countEmitted && m.emissive()) L += m.emission;
            if (depth + 1 > s.maxDepth) return L;
            glm::vec3 nn = h.n;
            float etaI = 1.0f, etaT = m.ior;
            float cosI = glm::dot(d, h.n);
            if (cosI > 0.0f) { nn = -h.n; std::swap(etaI, etaT); }   // ray inside → exiting
            cosI = std::fabs(cosI);
            const float eta = etaI / etaT;
            const float k = 1.0f - eta * eta * (1.0f - cosI * cosI);   // cos²θ_t
            const float Rtheta = (k < 0.0f) ? 1.0f : fresnelSchlick(cosI, etaI, etaT);
            if (k < 0.0f || rng() < Rtheta) {                          // total internal reflection or Fresnel reflect
                const glm::vec3 r = glm::reflect(d, nn);
                L += m.albedo * trace(h.p + nn * EPS, r, depth + 1, rng, true);
            } else {                                                   // refract
                const glm::vec3 t = eta * d + (eta * cosI - std::sqrt(k)) * nn;
                L += m.albedo * trace(h.p - nn * EPS, glm::normalize(t), depth + 1, rng, true);
            }
            return L;
        }

        // --- diffuse / glossy ---
        const glm::vec3 nn = (glm::dot(h.n, wo) < 0.0f) ? -h.n : h.n;
        if (countEmitted && m.emissive()) L += m.emission;
        L += directLight(h.p, nn, wo, m, rng);
        if (s.directLightingOnly) return L;

        if (depth + 1 > s.maxDepth) return L;
        if (rng() >= s.continuationProb) return L;                     // Russian roulette

        float pdf = 0.0f;
        const glm::vec3 wi = cosineSampleHemisphere(nn, rng(), rng(), pdf);
        if (pdf <= 0.0f) return L;
        const float cosTheta = std::max(0.0f, glm::dot(nn, wi));
        const glm::vec3 f = brdfEval(m, nn, wo, wi);
        // countEmitted=false: the next bounce's emission (if it hits a light) is already accounted
        // for by this vertex's NEE — avoids double counting.
        const glm::vec3 Li = trace(h.p + nn * EPS, wi, depth + 1, rng, false);
        L += f * Li * cosTheta / (pdf * s.continuationProb);
        return L;
    }
};

inline uint32_t seedFor(uint32_t x, uint32_t y, uint32_t base) {
    // splitmix-ish mix so neighboring pixels/seeds decorrelate.
    uint32_t h = x * 1973u + y * 9277u + base * 26699u + 1u;
    h ^= h >> 15; h *= 0x2c1b3c6du; h ^= h >> 12; h *= 0x297a2d39u; h ^= h >> 15;
    return h | 1u;
}

} // namespace

std::vector<glm::vec3> render(const Scene& scene, uint32_t width, uint32_t height,
                              const Settings& settings) {
    const glm::vec3 fwd   = glm::normalize(scene.camera.forward);
    const glm::vec3 right = glm::normalize(glm::cross(fwd, scene.camera.up));
    const glm::vec3 up    = glm::cross(right, fwd);
    const float aspect  = height ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
    const float tanHalf = std::tan(scene.camera.fovY * 0.5f);

    const Tracer tracer{scene, settings};
    std::vector<glm::vec3> out(static_cast<size_t>(width) * height, glm::vec3(0.0f));

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            Rng rng(seedFor(x, y, settings.seed));
            glm::vec3 acc(0.0f);
            for (int spp = 0; spp < settings.samplesPerPixel; ++spp) {
                const float jx = rng(), jy = rng();
                const float px = ((x + jx) / static_cast<float>(width))  * 2.0f - 1.0f;
                const float py = 1.0f - ((y + jy) / static_cast<float>(height)) * 2.0f;   // row 0 = top
                const glm::vec3 dir = glm::normalize(fwd
                                    + right * (px * aspect * tanHalf)
                                    + up    * (py * tanHalf));
                acc += tracer.trace(scene.camera.eye, dir, 0, rng, /*countEmitted=*/true);
            }
            acc /= static_cast<float>(std::max(1, settings.samplesPerPixel));
            out[static_cast<size_t>(y) * width + x] = glm::max(acc, glm::vec3(0.0f));   // clamp fireflies-below-0
        }
    }
    return out;
}

std::vector<uint8_t> toneMap(const std::vector<glm::vec3>& hdr, float exposure) {
    std::vector<uint8_t> out(hdr.size() * 4, 255);
    for (size_t i = 0; i < hdr.size(); ++i) {
        glm::vec3 c = hdr[i] * exposure;
        c = c / (glm::vec3(1.0f) + c);                       // Reinhard
        c = glm::pow(glm::clamp(c, 0.0f, 1.0f), glm::vec3(1.0f / 2.2f));   // gamma
        out[i * 4 + 0] = static_cast<uint8_t>(c.r * 255.0f + 0.5f);
        out[i * 4 + 1] = static_cast<uint8_t>(c.g * 255.0f + 0.5f);
        out[i * 4 + 2] = static_cast<uint8_t>(c.b * 255.0f + 0.5f);
        out[i * 4 + 3] = 255;
    }
    return out;
}

} // namespace engine::pt
