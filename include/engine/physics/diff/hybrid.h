//
//  hybrid.h
//  engine::physics::diff
//
//  α-order hybrid gradient estimator (Phase F3, after Suh et al. 2022, "Do Differentiable Simulators
//  Give Better Policy Gradients?"). It blends two unbiased estimators of the smoothed gradient
//  ∇f_σ(x) = ∇ E_{ε~N(0,I)}[f(x+σε)]:
//    • first-order (analytic):  Aᵢ = ∇f(x+σεᵢ)                       — low variance if f is smooth,
//                                                                       variance EXPLODES near stiff
//                                                                       / near-discontinuous regions.
//    • zeroth-order (ES):       Bᵢ = (f(x+σεᵢ) − f(x−σεᵢ))/(2σ) · εᵢ — bounded variance, robust to
//                                                                       discontinuity.
//  The combined estimator α·Ā + (1−α)·B̄ uses the minimum-variance convex weight
//    α* = (Var B − Cov(A,B)) / (Var A + Var B − 2 Cov(A,B))   (clamped to [0,1]),
//  so it leans on the cheap analytic gradient where the sim is smooth (α→1) and on the robust
//  zeroth-order estimate where contact makes it stiff (α→0). See
//  notes/investigations/2026-07-04-differentiable-reduced.md.
//

#pragma once

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

namespace engine::physics::diff {

struct AlphaGradResult {
    std::vector<double> grad;         // the α-order blended gradient
    std::vector<double> firstOrder;   // Ā  (pure analytic mean)
    std::vector<double> zeroth;       // B̄  (pure zeroth-order mean)
    double alpha = 0;                 // chosen blend weight
};

// f: value fn (const vector&)->double.  gradf: analytic gradient (const vector&)->vector<double>.
template <class F, class GradF>
AlphaGradResult alphaOrderGradient(F&& f, GradF&& gradf, const std::vector<double>& x,
                                   double sigma, int nSamples, uint64_t seed) {
    const size_t n = x.size();
    std::vector<double> A(static_cast<size_t>(nSamples) * n), B(static_cast<size_t>(nSamples) * n);
    std::vector<double> xp(n), xm(n), eps(n);
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> nd(0.0, 1.0);

    for (int s = 0; s < nSamples; ++s) {
        for (size_t i = 0; i < n; ++i) { eps[i] = nd(rng); xp[i] = x[i] + sigma * eps[i]; xm[i] = x[i] - sigma * eps[i]; }
        const std::vector<double> ga = gradf(xp);                       // first-order sample
        const double zc = (f(xp) - f(xm)) / (2.0 * sigma);              // zeroth-order coefficient
        for (size_t i = 0; i < n; ++i) { A[static_cast<size_t>(s) * n + i] = ga[i]; B[static_cast<size_t>(s) * n + i] = zc * eps[i]; }
    }

    AlphaGradResult r; r.firstOrder.assign(n, 0.0); r.zeroth.assign(n, 0.0); r.grad.assign(n, 0.0);
    for (int s = 0; s < nSamples; ++s) for (size_t i = 0; i < n; ++i) { r.firstOrder[i] += A[static_cast<size_t>(s) * n + i]; r.zeroth[i] += B[static_cast<size_t>(s) * n + i]; }
    for (size_t i = 0; i < n; ++i) { r.firstOrder[i] /= nSamples; r.zeroth[i] /= nSamples; }

    // Total (summed over components) sample variances + covariance of the two per-sample estimators.
    double varA = 0, varB = 0, cov = 0;
    for (int s = 0; s < nSamples; ++s) for (size_t i = 0; i < n; ++i) {
        const double da = A[static_cast<size_t>(s) * n + i] - r.firstOrder[i];
        const double db = B[static_cast<size_t>(s) * n + i] - r.zeroth[i];
        varA += da * da; varB += db * db; cov += da * db;
    }
    const double denom = varA + varB - 2.0 * cov;
    double alpha = (std::fabs(denom) < 1e-30) ? 0.5 : (varB - cov) / denom;
    alpha = std::clamp(alpha, 0.0, 1.0);
    r.alpha = alpha;
    for (size_t i = 0; i < n; ++i) r.grad[i] = alpha * r.firstOrder[i] + (1.0 - alpha) * r.zeroth[i];
    return r;
}

} // namespace engine::physics::diff
