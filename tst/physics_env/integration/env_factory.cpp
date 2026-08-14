#include "harness/harness.h"
//
//  env_factory.cpp
//  engine::tst / physics_env / integration
//
//  createVecEnv coherently selects the RL backend from EnvConfig::sim.backend, returning one IVecEnv:
//  Realtime/Reduced → VecEnv (PhysicsWorld), Diff → DiffVecEnv (CPU diff ABA), Cuda → CudaVecEnv (GPU;
//  requires ENGINE_CUDA — throws otherwise). Verifies the CPU backends build + step + share the same
//  flat obs/action contract (dims agree across backends for the same rig), and that Cuda throws in a
//  non-CUDA build. See 2026-07-08-cuda-engine-next-steps.md (#5, env selection/coherence).
//

#include <cmath>
#include <memory>
#include <stdexcept>

#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_env/env_factory.h"

using namespace engine;
using namespace engine::physics_env;

namespace {
EnvConfig cfg(physics::Backend backend) {
    EnvConfig c;
    c.articulation = physics::makeHumanoid();
    c.sim.backend = backend;
    c.sim.substeps = 8;
    c.sim.actionMode = physics::ActionMode::PDTarget;
    c.sim.kp = physics::Real(150); c.sim.kd = physics::Real(15); c.sim.maxTorque = physics::Real(150);
    return c;
}
bool stepsFinite(IVecEnv& e) {
    e.reset(0);
    for (float& v : e.actions()) v = 0.0f;
    for (int i = 0; i < 5; ++i) e.step();
    for (float v : e.observations()) if (!std::isfinite(v)) return false;
    return true;
}
} // namespace

TST_CASE(physics_env, integration, env_factory) {
    // Reduced (PhysicsWorld) and Diff (CPU diff ABA) both build + step through one IVecEnv handle.
    std::unique_ptr<IVecEnv> reduced = createVecEnv(4, cfg(physics::Backend::Reduced));
    std::unique_ptr<IVecEnv> diff    = createVecEnv(4, cfg(physics::Backend::Diff));
    TST_REQUIRE(reduced && diff);
    TST_REQUIRE(reduced->numEnvs() == 4 && diff->numEnvs() == 4);

    // Same rig ⇒ the flat action/obs contract agrees across backends (coherent selection).
    TST_REQUIRE_MSG(reduced->actDim() == diff->actDim(), "actDim must agree across backends for one rig");
    TST_REQUIRE_MSG(reduced->obsDim() == diff->obsDim(), "obsDim must agree across backends for one rig");

    TST_REQUIRE(stepsFinite(*reduced));
    TST_REQUIRE(stepsFinite(*diff));

    // Cuda in a non-ENGINE_CUDA build must throw (the GPU path isn't compiled here).
    bool threw = false;
    try { auto c = createVecEnv(4, cfg(physics::Backend::Cuda)); (void)c; }
    catch (const std::exception&) { threw = true; }
#if defined(ENGINE_CUDA)
    TST_REQUIRE(!threw);   // on a CUDA build it constructs
#else
    TST_REQUIRE_MSG(threw, "Backend::Cuda should throw without ENGINE_CUDA");
#endif
    std::printf("env_factory: reduced/diff actDim=%zu obsDim=%zu; cuda threw=%d\n",
                diff->actDim(), diff->obsDim(), threw ? 1 : 0);
}
