//
//  dual.h
//  engine::physics::diff
//
//  Forward-mode automatic differentiation (Milestone 2, Phase F1). `Dual<N>` carries a value plus N
//  first-order partial derivatives ("seeds") and overloads the usual arithmetic + transcendental
//  operations so it can be used as a drop-in scalar in templated numeric code — the code runs once
//  and the chain rule propagates exact derivatives w.r.t. the N seed variables.
//
//  We use forward mode because, per-step, the differentiable dynamics has #inputs ≈ #outputs
//  (state ≈ 2·ndof + action), so a full state-transition Jacobian costs the same order either way,
//  and forward mode is far simpler to build and validate. Downstream frameworks chain the per-step
//  Jacobians (their own BPTT). See notes/investigations/2026-07-04-differentiable-reduced.md.
//
//  Everything is double-precision: gradient/finite-difference agreement needs more than float.
//

#pragma once

#include <array>
#include <cmath>

namespace engine::physics::diff {

template <int N>
struct Dual {
    double v = 0;                 // value
    std::array<double, N> d{};    // partials ∂/∂(seedᵢ)

    Dual() = default;
    Dual(double value) : v(value) { d.fill(0.0); }                     // NOLINT: implicit constant
    Dual(double value, const std::array<double, N>& grad) : v(value), d(grad) {}

    // A variable seeded as the i-th independent input (∂self/∂seedᵢ = 1).
    static Dual seed(double value, int i) { Dual r(value); r.d[static_cast<size_t>(i)] = 1.0; return r; }
};

// ---- arithmetic (chain rule) ----
template <int N> Dual<N> operator-(const Dual<N>& a) {
    Dual<N> r; r.v = -a.v; for (int i = 0; i < N; ++i) r.d[i] = -a.d[i]; return r;
}
template <int N> Dual<N> operator+(const Dual<N>& a, const Dual<N>& b) {
    Dual<N> r; r.v = a.v + b.v; for (int i = 0; i < N; ++i) r.d[i] = a.d[i] + b.d[i]; return r;
}
template <int N> Dual<N> operator-(const Dual<N>& a, const Dual<N>& b) {
    Dual<N> r; r.v = a.v - b.v; for (int i = 0; i < N; ++i) r.d[i] = a.d[i] - b.d[i]; return r;
}
template <int N> Dual<N> operator*(const Dual<N>& a, const Dual<N>& b) {
    Dual<N> r; r.v = a.v * b.v; for (int i = 0; i < N; ++i) r.d[i] = a.d[i] * b.v + a.v * b.d[i]; return r;
}
template <int N> Dual<N> operator/(const Dual<N>& a, const Dual<N>& b) {
    Dual<N> r; r.v = a.v / b.v;
    const double inv = 1.0 / b.v, inv2 = inv * inv;
    for (int i = 0; i < N; ++i) r.d[i] = (a.d[i] * b.v - a.v * b.d[i]) * inv2;
    return r;
}
// Mixed Dual/scalar ops route through the implicit constant constructor, so `2.0 * x`,
// `x / h`, `x + 1.0` etc. all just work with zero-derivative constants.

template <int N> Dual<N>& operator+=(Dual<N>& a, const Dual<N>& b) { a = a + b; return a; }
template <int N> Dual<N>& operator-=(Dual<N>& a, const Dual<N>& b) { a = a - b; return a; }
template <int N> Dual<N>& operator*=(Dual<N>& a, const Dual<N>& b) { a = a * b; return a; }
template <int N> Dual<N>& operator/=(Dual<N>& a, const Dual<N>& b) { a = a / b; return a; }

// ---- comparisons (on the value; for branchy generic code — guards, sign tests, clamps) ----
template <int N> bool operator<(const Dual<N>& a, const Dual<N>& b) { return a.v < b.v; }
template <int N> bool operator>(const Dual<N>& a, const Dual<N>& b) { return a.v > b.v; }
template <int N> bool operator<=(const Dual<N>& a, const Dual<N>& b) { return a.v <= b.v; }
template <int N> bool operator>=(const Dual<N>& a, const Dual<N>& b) { return a.v >= b.v; }

// ---- transcendental / math (ADL-found; pair with `using std::sin;` in generic code) ----
template <int N> Dual<N> sin(const Dual<N>& a) {
    Dual<N> r; r.v = std::sin(a.v); const double c = std::cos(a.v);
    for (int i = 0; i < N; ++i) r.d[i] = c * a.d[i]; return r;
}
template <int N> Dual<N> cos(const Dual<N>& a) {
    Dual<N> r; r.v = std::cos(a.v); const double s = -std::sin(a.v);
    for (int i = 0; i < N; ++i) r.d[i] = s * a.d[i]; return r;
}
template <int N> Dual<N> sqrt(const Dual<N>& a) {
    Dual<N> r; r.v = std::sqrt(a.v); const double g = (a.v > 0.0) ? 0.5 / r.v : 0.0;
    for (int i = 0; i < N; ++i) r.d[i] = g * a.d[i]; return r;
}
template <int N> Dual<N> exp(const Dual<N>& a) {
    Dual<N> r; r.v = std::exp(a.v); for (int i = 0; i < N; ++i) r.d[i] = r.v * a.d[i]; return r;
}
template <int N> Dual<N> abs(const Dual<N>& a) {
    Dual<N> r; r.v = std::fabs(a.v); const double s = (a.v < 0.0) ? -1.0 : 1.0;
    for (int i = 0; i < N; ++i) r.d[i] = s * a.d[i]; return r;
}
template <int N> Dual<N> atan2(const Dual<N>& y, const Dual<N>& x) {
    Dual<N> r; r.v = std::atan2(y.v, x.v);
    const double den = x.v * x.v + y.v * y.v, inv = (den > 0.0) ? 1.0 / den : 0.0;
    for (int i = 0; i < N; ++i) r.d[i] = (x.v * y.d[i] - y.v * x.d[i]) * inv;   // d atan2 = (x dy − y dx)/(x²+y²)
    return r;
}

// ---- smooth activations (differentiable contact): logistic sigmoid + softplus ----------------
inline double sigmoid(double x) { return 1.0 / (1.0 + std::exp(-x)); }
inline double softplus(double x) { return x > 0.0 ? x + std::log1p(std::exp(-x)) : std::log1p(std::exp(x)); }   // numerically stable

template <int N> Dual<N> sigmoid(const Dual<N>& a) {
    Dual<N> r; const double s = sigmoid(a.v); r.v = s; const double g = s * (1.0 - s);
    for (int i = 0; i < N; ++i) r.d[i] = g * a.d[i]; return r;                  // σ' = σ(1−σ)
}
template <int N> Dual<N> softplus(const Dual<N>& a) {
    Dual<N> r; r.v = softplus(a.v); const double s = sigmoid(a.v);
    for (int i = 0; i < N; ++i) r.d[i] = s * a.d[i]; return r;                  // softplus' = σ
}

} // namespace engine::physics::diff
