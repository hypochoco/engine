# graphics
#
# The graphics module is an RHI (backend-agnostic interface) plus one backend
# implementation, selected at build time: Metal on Apple, Vulkan elsewhere. Only the
# chosen backend's sources are compiled. Backend-agnostic code lives in src/graphics/common.

file(GLOB_RECURSE GRAPHICS_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/*.hpp"
)

# Backend-agnostic sources (RHI-neutral: renderer, geometry store, ...).
file(GLOB_RECURSE GRAPHICS_COMMON_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/graphics/common/*.cpp"
)

# Per-backend sources — only one backend is compiled per build.
if(APPLE)
    file(GLOB_RECURSE GRAPHICS_BACKEND_SOURCE
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src/graphics/metal/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src/graphics/metal/*.mm"
    )
else()
    file(GLOB_RECURSE GRAPHICS_BACKEND_SOURCE
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src/graphics/vulkan/*.cpp"
    )
endif()

set(GRAPHICS_SOURCE ${GRAPHICS_COMMON_SOURCE} ${GRAPHICS_BACKEND_SOURCE})

add_library(engine_graphics STATIC
    ${GRAPHICS_HEADERS}
    ${GRAPHICS_SOURCE}
)
add_library(engine::graphics ALIAS engine_graphics)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/" PREFIX "Header Files" FILES ${GRAPHICS_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/graphics/" PREFIX "Source Files" FILES ${GRAPHICS_SOURCE})

target_include_directories(engine_graphics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

# dependencies shared by both backends
target_link_libraries(engine_graphics
    PUBLIC
        engine::core
        glfw
        glm::glm
        tinyobjloader
        stb::stb
)

if(APPLE)
    target_compile_definitions(engine_graphics PUBLIC ENGINE_RHI_METAL)
    # metal-cpp is header-only; add its root to the include path (Apple only).
    target_include_directories(engine_graphics PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/metal-cpp
    )
    target_link_libraries(engine_graphics
        PRIVATE
            ${METAL_FRAMEWORK}
            ${QUARTZ_FRAMEWORK}
            ${FOUNDATION_FRAMEWORK}
            ${COCOA_FRAMEWORK})
else()
    target_compile_definitions(engine_graphics PUBLIC ENGINE_RHI_VULKAN)
    target_link_libraries(engine_graphics
        PRIVATE
            Vulkan::Vulkan)
endif()
