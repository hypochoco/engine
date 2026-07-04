# controls
#
# Input-driven ECS controllers (e.g. FlyController). Consumes engine::input + engine::ecs +
# engine::core (Transform/Time) and drives entity Transforms. Deliberately graphics-free: it
# moves Transforms; the scene layer separately turns <Transform, Camera> into render views.

file(GLOB_RECURSE CONTROLS_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/controls/*.h"
)
file(GLOB_RECURSE CONTROLS_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/controls/*.cpp"
)

add_library(engine_controls STATIC
    ${CONTROLS_HEADERS}
    ${CONTROLS_SOURCE}
)
add_library(engine::controls ALIAS engine_controls)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/controls/" PREFIX "Header Files" FILES ${CONTROLS_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/controls/" PREFIX "Source Files" FILES ${CONTROLS_SOURCE})

target_include_directories(engine_controls
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_controls
    PUBLIC
        engine::ecs
        engine::input
        engine::core
)
