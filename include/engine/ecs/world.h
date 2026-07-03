//
//  world.h
//  engine::ecs
//
//  The World owns entities and archetypes. spawn<Ts...> creates an entity in the archetype
//  matching its component set; get/has access components; destroy swap-removes. query<Ts...>
//  (see query.h) iterates matching archetypes. Single-threaded; parallelism is across worlds.
//

#pragma once

#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "engine/ecs/archetype.h"
#include "engine/ecs/component.h"
#include "engine/ecs/entity.h"

namespace engine::ecs {

namespace detail {
inline uint32_t nextResourceId() { static uint32_t counter = 0; return counter++; }
}
template <class T>
uint32_t resourceId() { static const uint32_t id = detail::nextResourceId(); return id; }

template <class... Ts> class Query;   // query.h

class World {
public:
    template <class... Ts>
    Entity spawn(const Ts&... comps) {
        static_assert(sizeof...(Ts) >= 1, "spawn requires at least one component");
        std::array<ComponentInfo, sizeof...(Ts)> infos{ componentInfo<Ts>()... };
        std::sort(infos.begin(), infos.end(),
                  [](const ComponentInfo& a, const ComponentInfo& b) { return a.id < b.id; });

        const uint32_t archIdx = findOrCreateArchetype(infos.data(), infos.size());
        const Entity e = newEntity();
        Archetype& a = archetypes_[archIdx];
        const uint32_t row = a.addRowUninitialized(e);
        (writeComponent<Ts>(a, row, comps), ...);
        records_[e.index].archetype = archIdx;
        records_[e.index].row = row;
        return e;
    }

    template <class T>
    T* get(Entity e) {
        if (!alive(e)) return nullptr;
        const Record& rec = records_[e.index];
        Archetype& a = archetypes_[rec.archetype];
        const int col = a.columnIndex(componentId<std::remove_cvref_t<T>>());
        if (col < 0) return nullptr;
        return reinterpret_cast<T*>(a.columnPtr(col, rec.row));
    }

    template <class T>
    bool has(Entity e) {
        if (!alive(e)) return false;
        return archetypes_[records_[e.index].archetype].has(componentId<std::remove_cvref_t<T>>());
    }

    bool alive(Entity e) const {
        return e.valid() && e.index < records_.size() && records_[e.index].alive
            && records_[e.index].generation == e.generation;
    }

    void destroy(Entity e);
    size_t size() const { return liveCount_; }

    // --- resources (typed singletons: Time, camera, config, ...) ---
    template <class T>
    void setResource(T value) {
        resources_[resourceId<T>()] = std::make_shared<T>(std::move(value));
    }
    template <class T>
    T* getResource() {
        auto it = resources_.find(resourceId<T>());
        return it == resources_.end() ? nullptr : static_cast<T*>(it->second.get());
    }

    template <class... Ts> Query<Ts...> query();   // defined in query.h

    // Access for Query.
    std::vector<Archetype>& archetypes() { return archetypes_; }

private:
    struct Record {
        uint32_t generation = 0;
        uint32_t archetype  = 0;
        uint32_t row        = 0;
        bool     alive      = false;
    };

    std::vector<Record>    records_;
    std::vector<uint32_t>  freeIndices_;
    std::vector<Archetype> archetypes_;
    std::map<std::vector<ComponentId>, uint32_t> archetypeIndex_;   // ordered → deterministic
    std::unordered_map<uint32_t, std::shared_ptr<void>> resources_;
    size_t                 liveCount_ = 0;

    uint32_t findOrCreateArchetype(const ComponentInfo* infos, size_t n);   // world.cpp
    Entity   newEntity();                                                    // world.cpp

    template <class T>
    void writeComponent(Archetype& a, uint32_t row, const T& value) {
        const int col = a.columnIndex(componentId<T>());
        std::memcpy(a.columnPtr(col, row), &value, sizeof(T));
    }
};

} // namespace engine::ecs
