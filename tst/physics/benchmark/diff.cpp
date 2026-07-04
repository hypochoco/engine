//
//  diff.cpp
//  engine::tst / physics / benchmark
//
//  Phase F performance round: the cost of the differentiable path, so we know the overhead a
//  downstream trainer pays.
//    • forward differentiable substep (double) vs the production reduced backend substep;
//    • analytic gradient (rolloutGradient) cost vs the number of seeded action DOFs (forward-mode
//      scales ~linearly in seed count) against a plain forward rollout;
//    • per-step tangent Jacobian (54×75) cost for the humanoid.
//  Run optimized:  ./build/tst/benchmarks
//

#include <chrono>
#include <cstdio>
#include <vector>

#include "engine/physics/diff/diff_environment.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics/world.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics::diff;
using Clock = std::chrono::steady_clock;

TST_CASE(physics, benchmark, diff_forward_vs_reduced) {
    const physics::ArticulationDef def = physics::makeHumanoid();
    const DiffModel md = articulationToDiffModel(def);
    const V3<double> grav{ 0, -9.81, 0 }; const double h = 1.0 / 960.0;
    const std::vector<double> tau(static_cast<size_t>(md.ndofJoints), 0.0);

    DiffState<double> st = makeState<double>(md); st.basePos = { 0, 0.99, 0 };
    for (int i = 0; i < 500; ++i) diffSubstep(md, st, tau, grav, h);   // warm
    const int N = 50000;
    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) diffSubstep(md, st, tau, grav, h);
    const double diffMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / N;

    physics::WorldDef wd; wd.gravity = physics::Vec3(0, -9.81f, 0); wd.substeps = 1;
    auto world = physics::createPhysicsWorld(physics::Backend::Reduced, wd);
    physics::buildArticulation(*world, def);
    for (int i = 0; i < 200; ++i) world->step(static_cast<float>(h));
    const int M = 20000;
    t0 = Clock::now();
    for (int i = 0; i < M; ++i) world->step(static_cast<float>(h));
    const double redMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / M;

    std::printf("diff forward substep: diff(double)=%.4f ms  reduced(float)=%.4f ms  overhead=%.2fx  (%.0f substeps/s diff)\n",
                diffMs, redMs, diffMs / redMs, 1000.0 / diffMs);
}

TST_CASE(physics, benchmark, diff_gradient_cost) {
    DiffEnvironment env(physics::makeHumanoid());
    const int nSteps = 8;
    std::vector<double> action(static_cast<size_t>(env.actionDim()), 0.1);
    auto obj1 = [](const DiffState<Dual<1>>& s) { return s.qd[0]; };
    auto obj4 = [](const DiffState<Dual<4>>& s) { return s.qd[0]; };
    auto obj21 = [](const DiffState<Dual<21>>& s) { return s.qd[0]; };

    auto time = [&](auto fn) { fn(); const int R = 20; auto t0 = Clock::now(); for (int i = 0; i < R; ++i) fn();
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / R; };

    // forward-only baseline (double)
    const double fwd = time([&] { DiffState<double> s = env.state(); const V3<double> g{ 0, -9.81, 0 }; const double h = env.substepDt();
        for (int c = 0; c < nSteps; ++c) for (int i = 0; i < env.substeps(); ++i) diffSubstep(env.model(), s, action, g, h); (void)s; });
    const double g1 = time([&] { volatile double x = env.rolloutGradient<1>(action, nSteps, obj1)[0]; (void)x; });
    const double g4 = time([&] { volatile double x = env.rolloutGradient<4>(action, nSteps, obj4)[0]; (void)x; });
    const double g21 = time([&] { volatile double x = env.rolloutGradient<21>(action, nSteps, obj21)[0]; (void)x; });
    std::printf("diff gradient cost (%d control steps): forward=%.3f ms | grad NA=1:%.3f NA=4:%.3f NA=21:%.3f ms (%.1fx fwd)\n",
                nSteps, fwd, g1, g4, g21, g21 / fwd);
}

TST_CASE(physics, benchmark, diff_jacobian_cost) {
    DiffEnvironment env(physics::makeHumanoid());
    env.jacobian();
    const int R = 20; auto t0 = Clock::now();
    for (int i = 0; i < R; ++i) { volatile double x = env.jacobian().J[0]; (void)x; }
    const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / R;
    const StepJacobian J = env.jacobian();
    std::printf("diff per-step Jacobian (%dx%d, %d substeps): %.3f ms/step\n", J.nState, J.nInput, env.substeps(), ms);
}

// Contact cost vs number of contact points (differentiable engine only — the production backends are
// unaffected). Gates the contact-geometry work (features 2/3/4): ~per-contact-point cost + contact-on
// step cost. See notes/investigations/2026-07-04-differentiable-contact-geometry.md.
TST_CASE(physics, benchmark, diff_contact_cost) {
    const physics::ArticulationDef def = physics::makeHumanoid();
    const V3<double> grav{ 0, -9.81, 0 }; const double h = 1.0 / 960.0;

    auto substepMs = [&](DiffModel md, int& points) {
        points = 0;
        for (const auto& l : md.links) { if (l.contactRadius > 0.0) ++points; points += static_cast<int>(l.contactPoints.size()); }
        DiffState<double> st = makeState<double>(md); st.basePos = { 0, 0.99, 0 };
        const std::vector<double> tau(static_cast<size_t>(md.ndofJoints), 0.0);
        for (int i = 0; i < 500; ++i) diffSubstep(md, st, tau, grav, h);
        const int N = 60000; auto t0 = Clock::now();
        for (int i = 0; i < N; ++i) diffSubstep(md, st, tau, grav, h);
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / N;
    };

    DiffModel none = articulationToDiffModel(def);
    DiffModel feet = articulationToDiffModel(def, DiffContact::Feet);
    DiffModel all = articulationToDiffModel(def, DiffContact::All);
    int pn = 0, pf = 0, pa = 0;
    const double mn = substepMs(none, pn), mf = substepMs(feet, pf), ma = substepMs(all, pa);
    std::printf("diff contact cost: none=%.5f ms(%dpt)  feet=%.5f ms(%dpt)  all=%.5f ms(%dpt)  ~%.1f ns/point\n",
                mn, pn, mf, pf, ma, pa, (pa > pf ? (ma - mf) / (pa - pf) * 1e6 : 0.0));
}
