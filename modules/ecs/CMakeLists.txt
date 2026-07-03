# ecs
file(GLOB_RECURSE ECS_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/ecs/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/ecs/*.hpp"
)

file(GLOB_RECURSE ECS_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/ecs/*.cpp"
)

add_library(engine_ecs STATIC
    ${ECS_HEADERS}
    ${ECS_SOURCE}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/ecs/" PREFIX "Header Files" FILES ${ECS_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/ecs/" PREFIX "Source Files" FILES ${ECS_SOURCE})

add_library(engine::ecs ALIAS engine_ecs)

target_include_directories(engine_ecs
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

# dependencies (core provides Handle + Transform + glm transitively)
target_link_libraries(engine_ecs
    PUBLIC
        engine::core
)
