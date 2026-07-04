//
//  diff_hybrid.cpp
//  engine::tst / physics / unit
//
//  Phase F3a: validate the α-order hybrid gradient estimator (Suh et al. 2022). On a SMOOTH
//  objective the analytic first-order gradient is low-variance, so α→1 and the blend matches the
//  analytic gradient. On a STIFF / near-discontinuous objective the first-order gradient's variance
//  explodes, so α→0 and the α-order estimate has materially lower across-seed variance than pure
//  first-order — the defensible "differentiable sims don't always give better gradients" result.
//

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/articulated.h"
#include "engine/physics/diff/hybrid.h"
#include "harness/harness.h"

using namespace engine::physics::diff;

namespace {

template <class S> S angleZ(const M3<S>& R) { using std::atan2; return atan2(R.m[1][0], R.m[0][0]); }

DiffModel pendModel() {
    DiffModel md; md.ndofJoints = 1; md.floating = false;
    DiffLink base; base.parent = -1; base.mass = 1; base.Ic = diagM3(1, 1, 1); base.restRel = identity3<double>();
    DiffLink bob; bob.parent = 0; bob.dof = 1; bob.qIndex = 0; bob.mass = 1; bob.Ic = diagM3(0.001, 0.001, 0.001);
    bob.axes[0] = { 0, 0, 1 }; bob.anchorC = { -1, 0, 0 }; bob.restRel = rodrigues<double>(V3<double>{ 0, 0, 1 }, -M_PI / 2);
    md.links = { base, bob };
    return md;
}
template <class S>
S pendFinalAngle(const DiffModel& md, double q0, S tau, const V3<double>& g, double h, int steps) {
    DiffState<S> st = makeState<S>(md);
    st.linkRot[1] = rodrigues<S>(V3<S>{ S(0), S(0), S(1) }, S(q0));
    const std::vector<S> t{ tau };
    for (int i = 0; i < steps; ++i) diffSubstep(md, st, t, g, h);
    return angleZ(st.linkRot[1]);
}

} // namespace

// --- SMOOTH objective ⇒ α→1, blend matches the analytic gradient -------------------------------
TST_CASE(physics, unit, diff_hybrid_smooth_prefers_analytic) {
    const DiffModel md = pendModel();
    const V3<double> g{ 0, -9.81, 0 };
    const double h = 1.0 / 2000.0; const int steps = 150;
    const double q0 = 0.3, tau0 = 0.6;

    const double analytic = pendFinalAngle<Dual<1>>(md, q0, Dual<1>::seed(tau0, 0), g, h, steps).d[0];
    auto f = [&](const std::vector<double>& v) { return pendFinalAngle<double>(md, q0, v[0], g, h, steps); };
    auto gradf = [&](const std::vector<double>& v) {
        return std::vector<double>{ pendFinalAngle<Dual<1>>(md, q0, Dual<1>::seed(v[0], 0), g, h, steps).d[0] };
    };
    const auto r = alphaOrderGradient(f, gradf, std::vector<double>{ tau0 }, 1e-3, 4000, 5u);
    std::printf("hybrid_smooth: analytic=%.8f blended=%.8f alpha=%.3f\n", analytic, r.grad[0], r.alpha);
    TST_REQUIRE(r.alpha > 0.9);                                         // smooth ⇒ trust the analytic gradient
    TST_REQUIRE(std::fabs(r.grad[0] - analytic) / std::fabs(analytic) < 0.02);
}

// --- STIFF objective ⇒ α→0 and lower across-seed variance than pure first-order ----------------
TST_CASE(physics, unit, diff_hybrid_stiff_beats_first_order) {
    // f(x) = tanh(β x): a near-step at x=0. ∇f = β·sech²(βx) — huge near 0, ~0 away ⇒ the analytic
    // first-order estimator has exploding sample variance around the transition.
    const double beta = 200.0;
    auto f = [&](const std::vector<double>& v) { return std::tanh(beta * v[0]); };
    auto gradf = [&](const std::vector<double>& v) { const double c = std::cosh(beta * v[0]); return std::vector<double>{ beta / (c * c) }; };

    const std::vector<double> x{ 0.0 };                                // sit right on the transition
    const double sigma = 0.05; const int N = 400;

    // Across K seeds, compare the spread of the pure first-order mean vs the α-order estimate.
    const int K = 40; double mFO = 0, mFO2 = 0, mAO = 0, mAO2 = 0, alphaSum = 0;
    for (int k = 0; k < K; ++k) {
        const auto r = alphaOrderGradient(f, gradf, x, sigma, N, static_cast<uint64_t>(1000 + k));
        mFO += r.firstOrder[0]; mFO2 += r.firstOrder[0] * r.firstOrder[0];
        mAO += r.grad[0];       mAO2 += r.grad[0] * r.grad[0];
        alphaSum += r.alpha;
    }
    const double stdFO = std::sqrt(std::max(0.0, mFO2 / K - (mFO / K) * (mFO / K)));
    const double stdAO = std::sqrt(std::max(0.0, mAO2 / K - (mAO / K) * (mAO / K)));
    const double alphaAvg = alphaSum / K;
    std::printf("hybrid_stiff: alpha_avg=%.3f  std(firstOrder)=%.4f  std(alphaOrder)=%.4f  (ratio=%.2f)\n",
                alphaAvg, stdFO, stdAO, stdAO / stdFO);
    TST_REQUIRE(alphaAvg < 0.3);                                       // stiff ⇒ down-weight the analytic gradient
    TST_REQUIRE(stdAO < 0.6 * stdFO);                                  // hybrid is materially lower-variance
}
