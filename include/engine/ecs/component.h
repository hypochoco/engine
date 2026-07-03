//
//  component.h
//  engine::ecs
//
//  Compile-time component identity. Each component type T gets a stable ComponentId (a
//  process-global static counter) plus its size/alignment. Components must be trivially
//  copyable (rows are relocated between archetypes with memcpy).
//

#pragma once

#include <cstdint>
#include <type_traits>

namespace engine::ecs {

using ComponentId = uint32_t;

struct ComponentInfo {
    ComponentId id;
    uint32_t    size;
    uint32_t    align;
};

namespace detail {
inline ComponentId nextComponentId() {
    static ComponentId counter = 0;
    return counter++;
}
}

template <class T>
const ComponentInfo& componentInfo() {
    static_assert(std::is_trivially_copyable_v<T>,
                  "ECS components must be trivially copyable (rows are memcpy-relocated)");
    static const ComponentInfo info{
        detail::nextComponentId(),
        static_cast<uint32_t>(sizeof(T)),
        static_cast<uint32_t>(alignof(T)),
    };
    return info;
}

template <class T>
ComponentId componentId() { return componentInfo<T>().id; }

} // namespace engine::ecs
