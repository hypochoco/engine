
# core
file(GLOB_RECURSE CORE_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/core/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/core/*.hpp"
)

file(GLOB_RECURSE CORE_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/core/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/core/*.mm"
)

add_library(engine_core STATIC
    ${CORE_HEADERS}
    ${CORE_SOURCE}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/core/" PREFIX "Header Files" FILES ${CORE_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/core/" PREFIX "Source Files" FILES ${CORE_SOURCE})

add_library(engine::core ALIAS engine_core)

target_include_directories(engine_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

# dependencies
target_link_libraries(engine_core
    PUBLIC
        glm::glm
        Threads::Threads
)

# tinyobjloader (.obj mesh loader) and stb (image loader) are used only by the graphics module,
# not by core itself — link them only in a full (non-training) build so the headless training
# aggregate stays graphics-dependency-free.
if(NOT ENGINE_TRAINING_ONLY)
    target_link_libraries(engine_core PUBLIC tinyobjloader stb::stb)
    # Gates the asset-loader translation units (image.cpp / obj_loader.cpp) that need the
    # header-only stb + tinyobjloader. Headers stay dependency-free; a headless training build
    # (no stb/tinyobj) compiles the loaders to invalid-result stubs instead.
    target_compile_definitions(engine_core PUBLIC ENGINE_ASSET_LOADERS)
endif()
