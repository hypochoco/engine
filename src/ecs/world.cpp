//
//  world.cpp
//  engine::ecs
//
//  Non-template World internals: archetype find/create, entity allocation, destroy.
//

#include "engine/ecs/world.h"

#include <utility>

namespace engine::ecs {

uint32_t World::findOrCreateArchetype(const ComponentInfo* infos, size_t n) {
    std::vector<ComponentId> sig(n);
    for (size_t i = 0; i < n; ++i) sig[i] = infos[i].id;

    if (auto it = archetypeIndex_.find(sig); it != archetypeIndex_.end())
        return it->second;

    Archetype a;
    a.signature = sig;
    a.columns.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        Column c;
        c.id = infos[i].id;
        c.size = infos[i].size;
        a.columns.push_back(std::move(c));
    }
    const uint32_t idx = static_cast<uint32_t>(archetypes_.size());
    archetypes_.push_back(std::move(a));
    archetypeIndex_.emplace(std::move(sig), idx);
    return idx;
}

Entity World::newEntity() {
    uint32_t index;
    if (!freeIndices_.empty()) {
        index = freeIndices_.back();
        freeIndices_.pop_back();
    } else {
        index = static_cast<uint32_t>(records_.size());
        records_.push_back(Record{});
    }
    Record& r = records_[index];
    r.alive = true;
    ++liveCount_;
    return Entity{ index, r.generation };
}

void World::destroy(Entity e) {
    if (!alive(e)) return;
    Record& rec = records_[e.index];
    Archetype& a = archetypes_[rec.archetype];

    const Entity moved = a.removeRowSwap(rec.row);
    if (moved.valid()) records_[moved.index].row = rec.row;

    rec.alive = false;
    ++rec.generation;         // invalidate outstanding handles to this entity
    --liveCount_;
    freeIndices_.push_back(e.index);
}

} // namespace engine::ecs
