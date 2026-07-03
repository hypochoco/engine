# physics
#
# Phase-0 core: ECS-free, backend-agnostic (shapes, primitive collision, rigid-body state,
# integration kernels). Depends on engine::core only (glm). No ecs, no graphics.
# Plan: notes/investigations/2026-07-03-physics-plan.md.

file(GLOB_RECURSE PHYSICS_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics/*.hpp"
)

file(GLOB_RECURSE PHYSICS_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics/*.cpp"
)

add_library(engine_physics STATIC
    ${PHYSICS_HEADERS}
    ${PHYSICS_SOURCE}
)
add_library(engine::physics ALIAS engine_physics)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics/" PREFIX "Header Files" FILES ${PHYSICS_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics/" PREFIX "Source Files" FILES ${PHYSICS_SOURCE})

target_include_directories(engine_physics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_physics
    PUBLIC
        engine::core
)
