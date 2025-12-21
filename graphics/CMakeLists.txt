
# graphics
add_library(engine_graphics STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/graphics/graphics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/graphics/graphics_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/graphics/graphics_swapchain.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/graphics/graphics_texture.cpp
)

add_library(engine::graphics ALIAS engine_graphics)

target_include_directories(engine_graphics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)

# dependencies
target_link_libraries(engine_graphics
    PUBLIC
        glfw
        glm::glm
        Vulkan::Vulkan
        tinyobjloader
        stb::stb
)
