//
//  vec_env.cpp
//  engine::physics_env
//

#include "engine/physics_env/vec_env.h"

#include "engine/core/threading/thread_pool.h"

namespace engine::physics_env {

VecEnv::VecEnv(size_t numEnvs, const EnvConfig& config, engine::core::ThreadPool* pool)
    : pool_(pool) {
    envs_.reserve(numEnvs);
    for (size_t i = 0; i < numEnvs; ++i) envs_.push_back(std::make_unique<Environment>(config));
    actDim_ = envs_.empty() ? 0 : envs_.front()->actDim();
    obsDim_ = envs_.empty() ? 0 : envs_.front()->defaultObsDim();
    actions_.assign(numEnvs * actDim_, 0.0f);
    obs_.assign(numEnvs * obsDim_, 0.0f);
}

template <class F>
void VecEnv::forEachEnv(F&& f) {
    const size_t n = envs_.size();
    if (pool_ && n > 1) pool_->parallelFor(n, [&](std::size_t i) { f(i); }, 1);
    else for (size_t i = 0; i < n; ++i) f(i);
}

void VecEnv::packObs(size_t i) {
    envs_[i]->packDefaultObs(std::span<float>(obs_.data() + i * obsDim_, obsDim_));
}

void VecEnv::reset(uint64_t seed) {
    forEachEnv([&](size_t i) {
        envs_[i]->reset(seed + i);   // distinct per-env seed → reproducible + independent
        packObs(i);
    });
}

void VecEnv::resetMasked(std::span<const uint8_t> mask, uint64_t seed) {
    forEachEnv([&](size_t i) {
        if (i < mask.size() && mask[i]) { envs_[i]->reset(seed + i); packObs(i); }
    });
}

void VecEnv::step() {
    forEachEnv([&](size_t i) {
        envs_[i]->setAction(std::span<const float>(actions_.data() + i * actDim_, actDim_));
        envs_[i]->step();
        packObs(i);
    });
}

} // namespace engine::physics_env
