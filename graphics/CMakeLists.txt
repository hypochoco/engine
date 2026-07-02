
# graphics
file(GLOB_RECURSE GRAPHICS_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/engine/graphics/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/engine/graphics/*.hpp"
)

file(GLOB_RECURSE GRAPHICS_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/graphics/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/graphics/*.mm"
)

add_library(engine_graphics STATIC
    ${GRAPHICS_HEADERS}
    ${GRAPHICS_SOURCE}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include/engine/graphics/" PREFIX "Header Files" FILES ${GRAPHICS_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../src/graphics/" PREFIX "Source Files" FILES ${GRAPHICS_SOURCE})

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
        tinyobjloader
        stb::stb
)

if(APPLE)
    target_link_libraries(engine_graphics 
        PRIVATE 
            ${METAL_FRAMEWORK} 
            ${QUARTZ_FRAMEWORK})
else()
    target_link_libraries(engine_graphics 
        PRIVATE 
            Vulkan::Vulkan)
endif()
