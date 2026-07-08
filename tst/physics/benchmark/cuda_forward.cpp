//
//  cuda_forward.cpp
//  engine::tst / physics / benchmark
//
//  Phase 3 GPU micro-benchmark: the batched one-env-per-thread float forward kernel (BatchedForward)
//  on the humanoid with smoothed ground contact, at 4k / 16k / 64k envs — the A10G saturation sweep
//  the CUDA-port review asked for. Reports env-control-steps/s and env-substeps/s to compare against
//  the CPU baseline (reduced backend ~8k env-steps/s at substeps=48 on this box). Full CPU<->GPU
//  parity is Phase 5; here we assert only finiteness + a plausible root height (the sim didn't blow up).
//
//  Only built under ENGINE_CUDA (NVIDIA); a no-op TU otherwise.
//

#if defined(ENGINE_CUDA)

#include <chrono>
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
using Clock = std::chrono::steady_clock;

TST_CASE(physics, benchmark, cuda_batched_forward) {
    const physics::ArticulationDef def = physics::makeHumanoid();
    DiffModel md = articulationToDiffModel(def, DiffContact::All);   // smoothed ground contact (RL-like)
    const FlatModel fm = flatten(md);
    const V3<double> grav{ 0, -9.81, 0 };
    const int    substeps     = 48;             // RL contact-stability default
    const double controlDt    = 1.0 / 60.0;
    const double h            = controlDt / substeps;
    const int    controlSteps = 20;

    DiffState<float> s0 = makeState<float>(md);
    s0.basePos = { 0.0f, 0.99f, 0.0f };

    std::printf("cuda batched forward (humanoid %d links / %d dof, contact=All, substeps=%d):\n",
                fm.numLinks, fm.numDof, substeps);
    for (int N : { 4096, 16384, 65536 }) {
        engine::physics::cuda::BatchedForward sim(fm, N);
        std::vector<DiffState<float>> states(static_cast<size_t>(N), s0);
        sim.setStates(states);
        sim.setTau(std::vector<float>(static_cast<size_t>(N) * static_cast<size_t>(md.ndofJoints), 0.0f));

        sim.step(substeps, h, grav); sim.synchronize();   // warm (JIT/caches)
        sim.setStates(states);

        auto t0 = Clock::now();
        for (int c = 0; c < controlSteps; ++c) sim.step(substeps, h, grav);
        sim.synchronize();
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        const double envCtrlPerS = static_cast<double>(N) * controlSteps / (ms / 1000.0);
        const double envSubPerS  = envCtrlPerS * substeps;

        sim.getStates(states);
        bool finite = true; double meanY = 0.0;
        for (const auto& st : states) { finite = finite && std::isfinite(st.basePos.y); meanY += st.basePos.y; }
        meanY /= static_cast<double>(N);

        std::printf("  N=%6d  %7.1f ms  %10.0f env-ctrl-steps/s  %6.2fM env-substeps/s  meanRootY=%.3f  finite=%s\n",
                    N, ms, envCtrlPerS, envSubPerS / 1e6, meanY, finite ? "yes" : "NO");
        TST_REQUIRE(finite);
        TST_REQUIRE(meanY > -1.0 && meanY < 2.0);   // didn't explode / sink through the world
    }
}

#endif  // ENGINE_CUDA
