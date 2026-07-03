//
//  scheduler.h
//  engine::ecs
//
//  A Schedule is an ordered list of systems (each a `void(World&)` callable). Running the
//  schedule invokes them in insertion order — deterministic by construction. Systems read
//  shared state via World resources (e.g. Time{dt}) and iterate via queries.
//
//  Parallelism (across worlds, and later a read/write-declared within-world scheduler) is a
//  planned extension; this ordered form is the phase-1 scheduler.
//

#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace engine::ecs {

class World;

struct SystemDesc {
    std::string                    name;
    std::function<void(World&)>    fn;
};

class Schedule {
public:
    Schedule& add(std::string name, std::function<void(World&)> fn) {
        systems_.push_back({ std::move(name), std::move(fn) });
        return *this;
    }

    void run(World& world) const {
        for (const auto& s : systems_) s.fn(world);
    }

    size_t size() const { return systems_.size(); }
    const std::vector<SystemDesc>& systems() const { return systems_; }

private:
    std::vector<SystemDesc> systems_;
};

} // namespace engine::ecs
