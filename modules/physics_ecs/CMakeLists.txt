# physics_ecs
#
# The physics <-> ECS bridge: RigidBody component + step/sync systems. Depends on both
# engine::physics and engine::ecs (kept separate from engine::scene, which pulls graphics).

file(GLOB_RECURSE PHYSICS_ECS_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics_ecs/*.h"
)

file(GLOB_RECURSE PHYSICS_ECS_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics_ecs/*.cpp"
)

add_library(engine_physics_ecs STATIC
    ${PHYSICS_ECS_HEADERS}
    ${PHYSICS_ECS_SOURCE}
)
add_library(engine::physics_ecs ALIAS engine_physics_ecs)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics_ecs/" PREFIX "Header Files" FILES ${PHYSICS_ECS_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics_ecs/" PREFIX "Source Files" FILES ${PHYSICS_ECS_SOURCE})

target_include_directories(engine_physics_ecs
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_physics_ecs
    PUBLIC
        engine::physics
        engine::ecs
)
