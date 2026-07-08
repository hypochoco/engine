//
//  cuda_vec_env.cpp
//  engine::tst / physics_env / benchmark
//
//  Phase 4 validation + throughput for the GPU-resident CudaVecEnv. Confirms it matches the
//  physics_env::VecEnv/Environment contract (actDim / obsDim identical to the CPU Environment for the
//  same rig), steps with PD-target actuation, checks obs finiteness, and reports env-control-steps/s.
//  Only built under ENGINE_CUDA.
//

#if defined(ENGINE_CUDA)

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/physics_env/cuda_vec_env.h"
#include "engine/physics_env/environment.h"
#include "engine/physics/dynamics/articulation.h"
#include "harness/harness.h"

using namespace engine;
using Clock = std::chrono::steady_clock;

static physics_env::EnvConfig humanoidCfg() {
    physics_env::EnvConfig cfg;
    cfg.articulation      = physics::makeHumanoid();
    cfg.sim.substeps      = 48;
    cfg.sim.controlDt     = physics::Real(1) / physics::Real(60);
    cfg.sim.actionMode    = physics_env::ActionMode::PDTarget;
    cfg.sim.kp            = physics::Real(150);
    cfg.sim.kd            = physics::Real(15);
    cfg.sim.maxTorque     = physics::Real(150);
    cfg.sim.backend       = physics::Backend::Reduced;   // for the CPU reference Environment
    return cfg;
}

TST_CASE(physics_env, benchmark, cuda_vec_env) {
    const physics_env::EnvConfig cfg = humanoidCfg();

    // Contract reference from the CPU Environment.
    physics_env::Environment cpu(cfg);
    const size_t refAct = cpu.actDim();
    const size_t refObs = cpu.defaultObsDim();

    physics_env::CudaVecEnv sim(4096, cfg);
    std::printf("cuda_vec_env: actDim gpu=%zu cpu=%zu | obsDim gpu=%zu cpu=%zu\n",
                sim.actDim(), refAct, sim.obsDim(), refObs);
    TST_REQUIRE(sim.actDim() == refAct);
    TST_REQUIRE(sim.obsDim() == refObs);

    sim.reset(0);
    // zero PD targets (hold rest pose); step and confirm obs stays finite.
    auto act = sim.actions();
    for (float& a : act) a = 0.0f;
    for (int c = 0; c < 5; ++c) sim.step();
    auto obs = sim.observations();
    bool finite = true; for (float v : obs) finite = finite && std::isfinite(v);
    TST_REQUIRE(finite);
    std::printf("cuda_vec_env: rootY[0]=%.3f rootY[1]=%.3f obs finite=%s\n",
                obs[1], obs[static_cast<size_t>(sim.obsDim()) + 1], finite ? "yes" : "NO");

    // throughput
    for (int N : { 4096, 16384 }) {
        physics_env::CudaVecEnv s(static_cast<size_t>(N), cfg);
        s.reset(0);
        s.step();   // warm
        const int steps = 30;
        auto t0 = Clock::now();
        for (int c = 0; c < steps; ++c) s.step();
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        const double envCtrlPerS = static_cast<double>(N) * steps / (ms / 1000.0);
        std::printf("cuda_vec_env: N=%6d  %7.1f ms  %10.0f env-control-steps/s (incl. obs pack + host sync)\n",
                    N, ms, envCtrlPerS);
    }
}

#endif  // ENGINE_CUDA
