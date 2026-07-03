//
//  world.h
//  engine::physics
//
//  The runtime-virtual PhysicsWorld interface (physics plan §1, §4): a coarse boundary
//  (createBody / step / bulk readback) so multiple backends can coexist in one process, while
//  each backend's hot loops stay concrete and data-oriented. Phase-1 colliders are sphere +
//  plane; more shapes arrive with GJK/EPA in Phase 2.
//

#pragma once

#include <memory>
#include <span>

#include "engine/core/math/transform.h"
#include "engine/core/memory/handle.h"
#include "engine/physics/dynamics/body.h"
#include "engine/physics/shapes/shapes.h"
#include "engine/physics/types.h"

namespace engine::physics {

using BodyHandle = core::Handle<struct BodyTag>;

struct ColliderDesc {
    enum class Type { Sphere, Plane } type = Type::Sphere;
    Sphere sphere{};
    Plane  plane{};
};

struct BodyDef {
    Vec3            position{0};
    Quat            orientation{1, 0, 0, 0};
    Vec3            linearVelocity{0};
    Vec3            angularVelocity{0};
    Real            mass = Real(1);         // ignored for Static/Kinematic
    ColliderDesc    collider{};
    PhysicsMaterial material{};
    BodyType        type = BodyType::Dynamic;
};

struct WorldDef {
    Vec3 gravity{0, Real(-9.81), 0};
    int  velocityIterations = 8;
    int  substeps = 1;
};

struct ContactEvent {
    BodyHandle a, b;
    Vec3       point{0};
    Vec3       normal{0, 1, 0};
    Real       separation = 0;
};

class PhysicsWorld {
public:
    virtual ~PhysicsWorld() = default;

    virtual BodyHandle createBody(const BodyDef&) = 0;
    virtual void       destroyBody(BodyHandle)    = 0;
    virtual void       setGravity(Vec3)           = 0;
    virtual void       step(Real dt)              = 0;

    // Bulk, index-stable readback (indexed by BodyHandle.index) — one virtual call per array,
    // no per-body dispatch even at scale (§4).
    virtual std::span<const engine::Transform> poses() const              = 0;
    virtual std::span<const Vec3>               linearVelocities() const  = 0;
    virtual std::span<const Vec3>               angularVelocities() const = 0;

    // Single-body convenience.
    virtual engine::Transform pose(BodyHandle) const = 0;

    virtual std::span<const ContactEvent> contacts() const = 0;
};

enum class Backend { Realtime /* , Implicit (future) */ };

std::unique_ptr<PhysicsWorld> createPhysicsWorld(Backend backend, const WorldDef& def);

} // namespace engine::physics
