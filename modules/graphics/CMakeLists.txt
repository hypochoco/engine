# graphics
#
# Split into two targets so the graphics "head" is swappable:
#   engine::rhi    — the backend-agnostic GPU interface (rhi/) + the neutral, head-agnostic scene
#                    contract (view/) + the compiled backend (Metal on Apple, Vulkan elsewhere).
#                    The shared foundation both renderer heads stand on.
#   engine::render — the realtime renderer (render/): renderer, render graph, frame ring, geometry
#                    store. Depends on engine::rhi. A future path tracer is a sibling over rhi.
#
# The include tree stays include/engine/graphics/{rhi,view,render}; only the compiled units + link
# deps are partitioned. Only the chosen backend's sources are compiled.

# ---- headers (IDE/source_group only; headers don't affect compilation) ----
file(GLOB_RECURSE RHI_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/rhi/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/view/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/*.h"     # top-level (legacy graphics.h)
)
file(GLOB_RECURSE RENDER_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/render/*.h"
)

# ---- backend sources (only one backend per build) ----
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

# ---- realtime renderer sources (backend-agnostic) ----
file(GLOB_RECURSE RENDER_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/graphics/common/*.cpp"
)

# =====================================================================================
# engine::rhi — backend-agnostic GPU interface + neutral scene contract + backend impl
# =====================================================================================
add_library(engine_rhi STATIC
    ${RHI_HEADERS}
    ${GRAPHICS_BACKEND_SOURCE}
)
add_library(engine::rhi ALIAS engine_rhi)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/" PREFIX "Header Files" FILES ${RHI_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/graphics/" PREFIX "Source Files" FILES ${GRAPHICS_BACKEND_SOURCE})

target_include_directories(engine_rhi
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)
target_link_libraries(engine_rhi
    PUBLIC
        engine::core
        glfw
        glm::glm
)

if(APPLE)
    target_compile_definitions(engine_rhi PUBLIC ENGINE_RHI_METAL)
    # metal-cpp is header-only; add its root to the include path (Apple only).
    target_include_directories(engine_rhi PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/metal-cpp
    )
    target_link_libraries(engine_rhi
        PRIVATE
            ${METAL_FRAMEWORK}
            ${QUARTZ_FRAMEWORK}
            ${FOUNDATION_FRAMEWORK}
            ${COCOA_FRAMEWORK})
else()
    target_compile_definitions(engine_rhi PUBLIC ENGINE_RHI_VULKAN)
    target_link_libraries(engine_rhi
        PRIVATE
            Vulkan::Vulkan)
endif()

# =====================================================================================
# engine::render — the realtime renderer, over engine::rhi
# =====================================================================================
add_library(engine_render STATIC
    ${RENDER_HEADERS}
    ${RENDER_SOURCE}
)
add_library(engine::render ALIAS engine_render)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/graphics/render/" PREFIX "Header Files" FILES ${RENDER_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/graphics/common/" PREFIX "Source Files" FILES ${RENDER_SOURCE})

target_include_directories(engine_render
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)
target_link_libraries(engine_render
    PUBLIC
        engine::rhi
        engine::core
        glm::glm
)
