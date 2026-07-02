
# core
file(GLOB_RECURSE CORE_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/engine/core/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/engine/core/*.hpp"
)

file(GLOB_RECURSE CORE_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/core/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/core/*.mm"
)

add_library(engine_core STATIC
    ${CORE_HEADERS}
    ${CORE_SOURCE}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include/engine/core/" PREFIX "Header Files" FILES ${CORE_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../src/core/" PREFIX "Source Files" FILES ${CORE_SOURCE})

add_library(engine::core ALIAS engine_core)

target_include_directories(engine_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)

# dependencies
target_link_libraries(engine_core
    PUBLIC
        glm::glm
        tinyobjloader
        stb::stb
)
