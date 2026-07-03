//
//  physics.h
//  engine::physics
//
//  Umbrella include for the physics module (Phase 0: shapes, primitive collision, rigid-body
//  state, integration kernels). Design + phasing: notes/investigations/2026-07-03-physics-plan.md.
//  Phase-0 is the ECS-free, backend-agnostic core (depends on engine::core only).
//

#pragma once

#include "engine/physics/types.h"
#include "engine/physics/shapes/shapes.h"
#include "engine/physics/collision/contact.h"
#include "engine/physics/collision/primitives.h"
#include "engine/physics/dynamics/body.h"
#include "engine/physics/dynamics/integrate.h"
