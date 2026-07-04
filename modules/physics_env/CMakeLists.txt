# physics_env
#
# The RL env layer: headless Environment + VecEnv over a PhysicsWorld (Phase D). ECS-FREE by
# design — depends on engine::physics + engine::core only (NOT engine::ecs), so it stays lean for
# many-worlds training. Mirrors engine::physics_ecs (a physics + X bridge).

file(GLOB_RECURSE PHYSICS_ENV_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics_env/*.h"
)

file(GLOB_RECURSE PHYSICS_ENV_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics_env/*.cpp"
)

add_library(engine_physics_env STATIC
    ${PHYSICS_ENV_HEADERS}
    ${PHYSICS_ENV_SOURCE}
)
add_library(engine::physics_env ALIAS engine_physics_env)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics_env/" PREFIX "Header Files" FILES ${PHYSICS_ENV_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics_env/" PREFIX "Source Files" FILES ${PHYSICS_ENV_SOURCE})

target_include_directories(engine_physics_env
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_physics_env
    PUBLIC
        engine::physics
        engine::core
)
