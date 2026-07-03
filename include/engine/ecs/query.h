//
//  query.h
//  engine::ecs
//
//  Query<Ts...> iterates every archetype containing all of Ts and yields either per-entity
//  references (.each) or per-archetype contiguous spans (.chunks). `const T` in the query
//  marks read-only access. Iteration order is stable (archetype creation order, then row).
//

#pragma once

#include <cstdint>
#include <span>
#include <type_traits>
#include <utility>

#include "engine/ecs/archetype.h"
#include "engine/ecs/component.h"
#include "engine/ecs/entity.h"
#include "engine/ecs/world.h"

namespace engine::ecs {

template <class... Ts>
class Query {
public:
    explicit Query(World& world) : world_(&world) {}

    // fn(Entity, Ts&...)
    template <class F>
    void each(F&& fn) {
        for (Archetype& a : world_->archetypes()) {
            if (a.count == 0) continue;
            int cols[sizeof...(Ts)];
            if (!resolve(a, cols)) continue;
            for (uint32_t r = 0; r < a.count; ++r)
                dispatchEach(fn, a, r, cols, std::index_sequence_for<Ts...>{});
        }
    }

    // fn(std::span<Ts>...) — one call per matching archetype
    template <class F>
    void chunks(F&& fn) {
        for (Archetype& a : world_->archetypes()) {
            if (a.count == 0) continue;
            int cols[sizeof...(Ts)];
            if (!resolve(a, cols)) continue;
            dispatchChunk(fn, a, cols, std::index_sequence_for<Ts...>{});
        }
    }

private:
    World* world_;

    static bool resolve(Archetype& a, int (&cols)[sizeof...(Ts)]) {
        const ComponentId ids[] = { componentId<std::remove_cvref_t<Ts>>()... };
        for (size_t i = 0; i < sizeof...(Ts); ++i) {
            cols[i] = a.columnIndex(ids[i]);
            if (cols[i] < 0) return false;
        }
        return true;
    }

    template <class F, size_t... I>
    void dispatchEach(F& fn, Archetype& a, uint32_t r, int (&cols)[sizeof...(Ts)],
                      std::index_sequence<I...>) {
        fn(a.entities[r], *reinterpret_cast<Ts*>(a.columnPtr(cols[I], r))...);
    }

    template <class F, size_t... I>
    void dispatchChunk(F& fn, Archetype& a, int (&cols)[sizeof...(Ts)],
                       std::index_sequence<I...>) {
        fn(std::span<Ts>(reinterpret_cast<Ts*>(a.columnPtr(cols[I], 0)), a.count)...);
    }
};

template <class... Ts>
Query<Ts...> World::query() { return Query<Ts...>(*this); }

} // namespace engine::ecs
