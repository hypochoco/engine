# pathtracer
#
# The offline path-tracer renderer head (step 1: a CPU reference integrator). Pure CPU + glm —
# depends on engine::core only (NO Eigen, NO RHI yet). Brute-force intersection; a data-oriented
# BVH and a GPU (Slang compute) path come later. A sibling head to engine::render; both will consume
# the neutral render::RenderView. See notes/investigations/path-tracing/.

file(GLOB_RECURSE PATHTRACER_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/pathtracer/*.h"
)

file(GLOB_RECURSE PATHTRACER_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/pathtracer/*.cpp"
)

add_library(engine_pathtracer STATIC
    ${PATHTRACER_HEADERS}
    ${PATHTRACER_SOURCE}
)
add_library(engine::pathtracer ALIAS engine_pathtracer)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/pathtracer/" PREFIX "Header Files" FILES ${PATHTRACER_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/pathtracer/" PREFIX "Source Files" FILES ${PATHTRACER_SOURCE})

target_include_directories(engine_pathtracer
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_pathtracer
    PUBLIC
        engine::core
        glm::glm
)
