//
//  archetype.h
//  engine::ecs
//
//  An archetype (table) stores all entities sharing the same set of component types. Each
//  component is a contiguous column (SoA); row r holds one entity's components across all
//  columns. Structural removal is swap-with-last (O(1)); rows relocate via memcpy.
//

#pragma once

#include <cstddef>
#include <cstring>
#include <vector>

#include "engine/ecs/component.h"
#include "engine/ecs/entity.h"

namespace engine::ecs {

struct Column {
    ComponentId            id   = 0;
    uint32_t               size = 0;   // bytes per element
    std::vector<std::byte> data;
};

struct Archetype {
    std::vector<ComponentId> signature;   // sorted component ids
    std::vector<Column>      columns;      // parallel to `signature`
    std::vector<Entity>      entities;     // row -> entity
    uint32_t                 count = 0;

    int columnIndex(ComponentId cid) const {
        for (size_t i = 0; i < signature.size(); ++i)
            if (signature[i] == cid) return static_cast<int>(i);
        return -1;
    }
    bool has(ComponentId cid) const { return columnIndex(cid) >= 0; }

    void* columnPtr(int col, uint32_t row) {
        Column& c = columns[static_cast<size_t>(col)];
        return c.data.data() + static_cast<size_t>(row) * c.size;
    }

    // Grows every column by one (uninitialized) element; returns the new row index.
    uint32_t addRowUninitialized(Entity e) {
        for (Column& c : columns) c.data.resize(c.data.size() + c.size);
        entities.push_back(e);
        return count++;
    }

    // Swap-remove: moves the last row into `row`. Returns the entity that was moved (invalid
    // if `row` was already the last row) so the caller can fix up its location record.
    Entity removeRowSwap(uint32_t row) {
        const uint32_t last = count - 1;
        Entity moved{};
        if (row != last) {
            for (Column& c : columns) {
                std::memcpy(c.data.data() + static_cast<size_t>(row) * c.size,
                            c.data.data() + static_cast<size_t>(last) * c.size, c.size);
            }
            entities[row] = entities[last];
            moved = entities[row];
        }
        for (Column& c : columns) c.data.resize(c.data.size() - c.size);
        entities.pop_back();
        --count;
        return moved;
    }
};

} // namespace engine::ecs
