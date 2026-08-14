//
//  env_factory.cpp
//  engine::physics_env
//

#include "engine/physics_env/env_factory.h"

#include <stdexcept>

#include "engine/physics_env/diff_vec_env.h"
#include "engine/physics_env/vec_env.h"

#if defined(ENGINE_CUDA)
#include "engine/physics_env/cuda_vec_env.h"
#endif

namespace engine::physics_env {

std::unique_ptr<IVecEnv> createVecEnv(size_t numEnvs, const EnvConfig& config,
                                      engine::core::ThreadPool* pool) {
    switch (config.sim.backend) {
        case physics::Backend::Realtime:
        case physics::Backend::Reduced:
            return std::make_unique<VecEnv>(numEnvs, config, pool);   // PhysicsWorld (maximal / reduced PGS)
        case physics::Backend::Diff:
            return std::make_unique<DiffVecEnv>(numEnvs, config, pool);   // CPU differentiable ABA
        case physics::Backend::Cuda:
#if defined(ENGINE_CUDA)
            return std::make_unique<CudaVecEnv>(numEnvs, config);         // GPU differentiable ABA (no pool)
#else
            throw std::runtime_error(
                "createVecEnv: Backend::Cuda requires an ENGINE_CUDA build (NVIDIA only); "
                "this build has no CUDA physics backend.");
#endif
    }
    throw std::runtime_error("createVecEnv: unknown physics::Backend");
}

} // namespace engine::physics_env
