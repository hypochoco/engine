//
//  zeroth_order.h
//  engine::physics::diff
//
//  Zeroth-order (randomized-smoothing / evolution-strategies) gradient estimator (Phase Fd). It
//  estimates the gradient of the Gaussian-SMOOTHED objective f_σ(x) = E_{ε~N(0,I)}[f(x+σε)] using
//  only forward evaluations of f — no derivatives, so it works through non-smooth / discontinuous f
//  (it differentiates the smoothed function, which is well-defined even where f has kinks). This is
//  the robust baseline for differentiable-sim policy gradients and the zeroth-order half of the
//  α-order hybrid (F3); on smooth f it agrees with the analytic `Dual` gradient.
//
//  Antithetic central-difference form (variance-reduced):
//      ∇f_σ(x) ≈ (1/N) Σᵢ [ (f(x+σεᵢ) − f(x−σεᵢ)) / (2σ) ] εᵢ.
//  Deterministic given `seed`. The N evaluations are embarrassingly parallel — downstream they map
//  onto the fast VecEnv (each perturbed rollout is one env). See
//  notes/investigations/2026-07-04-differentiable-reduced.md.
//

#pragma once

#include <cstdint>
#include <random>
#include <vector>

namespace engine::physics::diff {

// f: (const std::vector<double>&) -> double. Returns an n-vector estimate of ∇f_σ(x).
template <class F>
std::vector<double> zerothOrderGradient(F&& f, const std::vector<double>& x, double sigma,
                                        int nSamples, uint64_t seed) {
    const size_t n = x.size();
    std::vector<double> g(n, 0.0), xp(n), xm(n), eps(n);
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> nd(0.0, 1.0);
    for (int s = 0; s < nSamples; ++s) {
        for (size_t i = 0; i < n; ++i) { eps[i] = nd(rng); xp[i] = x[i] + sigma * eps[i]; xm[i] = x[i] - sigma * eps[i]; }
        const double coeff = (f(xp) - f(xm)) / (2.0 * sigma);
        for (size_t i = 0; i < n; ++i) g[i] += coeff * eps[i];
    }
    for (size_t i = 0; i < n; ++i) g[i] /= static_cast<double>(nSamples);
    return g;
}

} // namespace engine::physics::diff
