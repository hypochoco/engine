//
//  env_factory.h
//  engine::physics_env
//
//  One entry point that turns an EnvConfig into the RL VecEnv for its `sim.backend`, so a single
//  config coherently selects any backend and callers hold one `IVecEnv`:
//    Realtime / Reduced → VecEnv      (maximal / reduced-coordinate PhysicsWorld)
//    Diff               → DiffVecEnv  (CPU differentiable ABA + smoothed contact)
//    Cuda               → CudaVecEnv  (GPU differentiable ABA; requires an ENGINE_CUDA build)
//  Diff and Cuda run the SAME templated diff ABA (same physics, CPU vs GPU); Realtime/Reduced are the
//  PhysicsWorld backends. Requesting Cuda in a non-ENGINE_CUDA build throws (the GPU path isn't compiled).
//

#pragma once

#include <cstddef>
#include <memory>

#include "engine/physics_env/environment.h"     // EnvConfig
#include "engine/physics_env/vec_env_base.h"    // IVecEnv

namespace engine::core { class ThreadPool; }

namespace engine::physics_env {

// Construct the vectorized env selected by `config.sim.backend`. `pool` parallelizes the CPU backends
// (VecEnv / DiffVecEnv) across worlds; it is ignored by CudaVecEnv (the GPU parallelizes envs itself).
std::unique_ptr<IVecEnv> createVecEnv(size_t numEnvs, const EnvConfig& config,
                                      engine::core::ThreadPool* pool = nullptr);

} // namespace engine::physics_env
