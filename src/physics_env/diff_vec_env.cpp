//
//  diff_vec_env.cpp
//  engine::physics_env
//

#include "engine/physics_env/diff_vec_env.h"

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/diff/env_ops.h"
#include "engine/physics/diff/from_articulation.h"

namespace engine::physics_env {

DiffVecEnv::DiffVecEnv(size_t numEnvs, const EnvConfig& config, engine::core::ThreadPool* pool)
    : pool_(pool) {
    const physics::SimConfig& sim = config.sim;

    // Build the differentiable model (shape-aware ground contact on every body, matching CudaVecEnv).
    model_    = diff::articulationToDiffModel(config.articulation, diff::DiffContact::All);
    numLinks_ = static_cast<int>(model_.links.size());
    numDof_   = model_.ndofJoints;
    actDim_   = static_cast<size_t>(numDof_);
    obsDim_   = static_cast<size_t>(13 + 2 * numDof_ + numLinks_);

    substeps_   = sim.substeps > 0 ? sim.substeps : 1;
    h_          = static_cast<double>(sim.controlDt) / substeps_;
    gravity_    = { sim.gravity.x, sim.gravity.y, sim.gravity.z };
    actionMode_ = (sim.actionMode == physics::ActionMode::PDTarget) ? diff::kActionPDTarget : diff::kActionTorque;
    kp_ = static_cast<float>(sim.kp); kd_ = static_cast<float>(sim.kd); maxTorque_ = static_cast<float>(sim.maxTorque);

    // Authored initial state: root pose from the articulation's root body, rest joints, zero velocity.
    const int root = diff::rootBodyIndex(model_);
    const physics::BodyDef& rb = config.articulation.bodies[static_cast<size_t>(root)];
    init_ = diff::makeState<float>(model_);
    init_.basePos = { static_cast<float>(rb.position.x), static_cast<float>(rb.position.y), static_cast<float>(rb.position.z) };
    init_.baseRot = diff::lift<float>(diff::glmToM3(glm::mat3_cast(rb.orientation)));

    states_.assign(numEnvs, init_);
    actions_.assign(numEnvs * actDim_, 0.0f);
    obs_.assign(numEnvs * obsDim_, 0.0f);

    reset(0);
}

template <class F>
void DiffVecEnv::forEachEnv(F&& f) {
    const size_t n = states_.size();
    if (pool_ && n > 1) pool_->parallelFor(n, [&](std::size_t i) { f(i); }, 1);
    else for (size_t i = 0; i < n; ++i) f(i);
}

void DiffVecEnv::packObs(size_t i) {
    diff::packDefaultObs(model_, states_[i], obs_.data() + i * obsDim_);
}

void DiffVecEnv::reset(uint64_t /*seed*/) {
    // Deterministic authored init for every env (matches physics_env::VecEnv with no reset hook;
    // per-env domain randomization is a future hook — see engine next-steps note).
    forEachEnv([&](size_t i) { states_[i] = init_; packObs(i); });
}

void DiffVecEnv::resetMasked(std::span<const uint8_t> mask, uint64_t /*seed*/) {
    forEachEnv([&](size_t i) {
        if (i < mask.size() && mask[i]) { states_[i] = init_; packObs(i); }
    });
}

void DiffVecEnv::step() {
    forEachEnv([&](size_t i) {
        float tau[diff::kMaxDof] = {};                       // per-env scratch (thread-local on the pool)
        const float* action = actions_.data() + i * actDim_;
        diff::actionToTau(model_, states_[i], action, tau, actionMode_, kp_, kd_, maxTorque_);
        for (int s = 0; s < substeps_; ++s) diff::diffSubstep(model_, states_[i], tau, gravity_, h_);
        packObs(i);
    });
}

} // namespace engine::physics_env
