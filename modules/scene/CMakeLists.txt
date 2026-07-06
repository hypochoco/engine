# scene
#
# The ECS <-> render bridge: render components + the extraction system. Depends on engine::ecs and
# engine::rhi (the neutral scene contract in graphics/view/ — NOT a specific renderer head, so this
# bridge stays head-agnostic; the realtime renderer and a future path tracer both consume its output).

file(GLOB_RECURSE SCENE_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/scene/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/scene/*.hpp"
)

file(GLOB_RECURSE SCENE_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/scene/*.cpp"
)

add_library(engine_scene STATIC
    ${SCENE_HEADERS}
    ${SCENE_SOURCE}
)
add_library(engine::scene ALIAS engine_scene)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/scene/" PREFIX "Header Files" FILES ${SCENE_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/scene/" PREFIX "Source Files" FILES ${SCENE_SOURCE})

target_include_directories(engine_scene
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_scene
    PUBLIC
        engine::ecs
        engine::rhi
)
