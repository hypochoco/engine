//
//  cuda_parity.cpp
//  engine::tst / physics / integration
//
//  Phase 5 of the CUDA port: correctness gates for the batched GPU forward path.
//   - cuda_diff_parity: the GPU float kernel vs the SAME templated diffSubstep on the CPU. Compared
//     at matched precision (CPU float vs GPU float — only FMA/rounding order differs ⇒ tight), and
//     against the CPU double oracle (float approximation ⇒ looser). Reuses the diff ABA as its own
//     oracle, per the port plan (parity is tolerance-based, never bit-exact across CPU/GPU).
//   - cuda_determinism: same-device determinism — run-to-run bit-identical, and (identical init ⇒)
//     env-to-env bit-identical, since the kernel has no cross-env atomics/reductions.
//  Only built under ENGINE_CUDA.
//

#if defined(ENGINE_CUDA)

#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics/cuda/batched_forward.h"
#include "engine/physics/diff/flat_model.h"
#include "engine/physics/diff/from_articulation.h"
#include "engine/physics/dynamics/articulation.h"
#include "harness/harness.h"

using namespace engine;
using namespace engine::physics::diff;

namespace {

DiffState<double> makeInit(const DiffModel& md) {
    DiffState<double> st = makeState<double>(md);
    st.basePos = { 0.0, 1.05, 0.0 };
    for (int k = 0; k < st.numDof; ++k) st.qd[k] = 0.05 * ((k % 3) - 1);   // gentle non-trivial motion
    return st;
}

template <class S> double maxPosDiff(const DiffState<S>& a, const V3<double>& b) {
    return std::max({ std::fabs((double)a.basePos.x - b.x),
                      std::fabs((double)a.basePos.y - b.y),
                      std::fabs((double)a.basePos.z - b.z) });
}
template <class A, class B> double maxQdDiff(const A& a, const B& b, int n) {
    double m = 0; for (int k = 0; k < n; ++k) m = std::max(m, std::fabs((double)a.qd[k] - (double)b.qd[k])); return m;
}

} // namespace

TST_CASE(physics, integration, cuda_diff_parity) {
    const physics::ArticulationDef def = physics::makeHumanoid();
    DiffModel md = articulationToDiffModel(def, DiffContact::All);
    const FlatModel fm = flatten(md);
    const V3<double> grav{ 0, -9.81, 0 };
    const double h = 1.0 / (60.0 * 48.0);
    const int K = 16;

    std::vector<double> tau_d(static_cast<size_t>(md.ndofJoints));
    std::vector<float>  tau_f(static_cast<size_t>(md.ndofJoints));
    for (int k = 0; k < md.ndofJoints; ++k) { tau_d[k] = 0.5 * std::sin(0.7 * k); tau_f[k] = (float)tau_d[k]; }

    const DiffState<double> init = makeInit(md);

    // CPU references: same-precision float, and the double oracle.
    DiffState<float>  st_cpu_f = liftState<float>(init);
    DiffState<double> st_cpu_d = init;
    for (int i = 0; i < K; ++i) diffSubstep(md, st_cpu_f, tau_f, grav, h);
    for (int i = 0; i < K; ++i) diffSubstep(md, st_cpu_d, tau_d, grav, h);

    // GPU float.
    engine::physics::cuda::BatchedForward sim(fm, 1);
    sim.setStates({ liftState<float>(init) });
    sim.setTau(tau_f);
    sim.step(K, h, grav);
    std::vector<DiffState<float>> out; sim.getStates(out);
    const DiffState<float>& st_gpu = out[0];

    const V3<double> cpuf{ st_cpu_f.basePos.x, st_cpu_f.basePos.y, st_cpu_f.basePos.z };
    const V3<double> cpud{ st_cpu_d.basePos.x, st_cpu_d.basePos.y, st_cpu_d.basePos.z };
    const double posFF = maxPosDiff(st_gpu, cpuf);   // GPU float vs CPU float (tight)
    const double qdFF  = maxQdDiff(st_gpu, st_cpu_f, md.ndofJoints);
    const double posFD = maxPosDiff(st_gpu, cpud);   // GPU float vs CPU double (looser)
    const double qdFD  = maxQdDiff(st_gpu, st_cpu_d, md.ndofJoints);

    std::printf("cuda_diff_parity (K=%d substeps, contact=All): GPUf-vs-CPUf pos=%.2e qd=%.2e | GPUf-vs-CPUd pos=%.2e qd=%.2e\n",
                K, posFF, qdFF, posFD, qdFD);

    // Same-precision CPU vs GPU: only FMA/rounding-order differences ⇒ tight.
    TST_REQUIRE(posFF < 1e-3);
    TST_REQUIRE(qdFF  < 1e-2);
    // Float vs the double oracle: float truncation over K stiff-contact substeps ⇒ looser but small.
    TST_REQUIRE(posFD < 2e-2);
    TST_REQUIRE(qdFD  < 2e-1);
}

TST_CASE(physics, integration, cuda_determinism) {
    const physics::ArticulationDef def = physics::makeHumanoid();
    DiffModel md = articulationToDiffModel(def, DiffContact::All);
    const FlatModel fm = flatten(md);
    const V3<double> grav{ 0, -9.81, 0 };
    const double h = 1.0 / (60.0 * 48.0);
    const int N = 1024;

    const DiffState<float> init = liftState<float>(makeInit(md));
    std::vector<DiffState<float>> states(static_cast<size_t>(N), init);   // identical init across envs
    std::vector<float> tau(static_cast<size_t>(N) * static_cast<size_t>(md.ndofJoints));
    for (int e = 0; e < N; ++e) for (int k = 0; k < md.ndofJoints; ++k) tau[e * md.ndofJoints + k] = 0.5f * std::sin(0.7f * k);

    auto run = [&]() {
        engine::physics::cuda::BatchedForward sim(fm, N);
        sim.setStates(states); sim.setTau(tau); sim.step(48, h, grav);
        std::vector<DiffState<float>> out; sim.getStates(out); return out;
    };
    const std::vector<DiffState<float>> a = run();
    const std::vector<DiffState<float>> b = run();

    // run-to-run: bit-identical (same device, no atomics/reductions).
    bool runEqual = true;
    for (int e = 0; e < N && runEqual; ++e) {
        runEqual = (a[e].basePos.y == b[e].basePos.y) && (a[e].basePos.x == b[e].basePos.x) && (a[e].basePos.z == b[e].basePos.z);
        for (int k = 0; k < md.ndofJoints && runEqual; ++k) runEqual = (a[e].qd[k] == b[e].qd[k]);
    }
    // env-to-env: identical init ⇒ identical result.
    bool envEqual = (a[0].basePos.y == a[N - 1].basePos.y);
    for (int k = 0; k < md.ndofJoints && envEqual; ++k) envEqual = (a[0].qd[k] == a[N - 1].qd[k]);

    std::printf("cuda_determinism (N=%d, 48 substeps): run-to-run bit-identical=%s  env-to-env bit-identical=%s  rootY=%.6f\n",
                N, runEqual ? "yes" : "NO", envEqual ? "yes" : "NO", a[0].basePos.y);
    TST_REQUIRE(runEqual);
    TST_REQUIRE(envEqual);
}

#endif  // ENGINE_CUDA
