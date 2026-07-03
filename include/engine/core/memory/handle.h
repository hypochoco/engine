//
//  handle.h
//  engine::core / memory
//
//  Generational handle: a small value type identifying a slot in some owner's pool. The
//  `generation` guards against using a handle whose slot was freed and reused (stale-handle
//  / use-after-free detection). Shared by the RHI (GPU resources) and the ECS (entities) —
//  promoted here once there were two consumers (see notes/investigations geometry-scaling +
//  ecs plan).
//

#pragma once

#include <cstdint>

namespace engine::core {

template <class Tag>
struct Handle {
    static constexpr uint32_t kInvalid = 0xFFFF'FFFFu;

    uint32_t index      = kInvalid;
    uint32_t generation = 0;

    constexpr bool valid() const { return index != kInvalid; }
    constexpr bool operator==(const Handle&) const = default;
};

} // namespace engine::core
