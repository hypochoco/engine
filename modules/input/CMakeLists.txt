# input
#
# The neutral, backend-agnostic input layer: `InputState` + Key/MouseButton. Header-only and
# GLFW-free (depends only on core/glm) so it works headless or with scripted/ML input, and any
# windowing backend can fill it. GLFW-specific polling lives in engine::input_glfw; a future
# engine::input_qt would sit alongside it.

add_library(engine_input INTERFACE)
add_library(engine::input ALIAS engine_input)

target_include_directories(engine_input
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_input
    INTERFACE
        engine::core
        glm::glm
)
