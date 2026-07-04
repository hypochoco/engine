# input_glfw
#
# GLFW adapter for engine::input: polls a GLFW window into an InputState. Isolated in its own
# target so the neutral engine::input stays GLFW-free (a future engine::input_qt lives here too).

file(GLOB_RECURSE INPUT_GLFW_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/input_glfw/*.h"
)
file(GLOB_RECURSE INPUT_GLFW_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/input_glfw/*.cpp"
)

add_library(engine_input_glfw STATIC
    ${INPUT_GLFW_HEADERS}
    ${INPUT_GLFW_SOURCE}
)
add_library(engine::input_glfw ALIAS engine_input_glfw)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/input_glfw/" PREFIX "Header Files" FILES ${INPUT_GLFW_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/input_glfw/" PREFIX "Source Files" FILES ${INPUT_GLFW_SOURCE})

target_include_directories(engine_input_glfw
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_input_glfw
    PUBLIC
        engine::input
        glfw
)
