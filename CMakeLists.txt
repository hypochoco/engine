
cmake_minimum_required(VERSION 3.25)
project(engine VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to an OPTIMIZED build when the user doesn't specify one. Without this a plain
# `cmake -S . -B build` yields an unoptimized (no -O, no NDEBUG) build — physics is ~20-50x
# slower there, which makes the interactive visuals run in slow motion. Single-config generators
# only (multi-config like Xcode/VS pick per-build). Override with -DCMAKE_BUILD_TYPE=Debug.
get_property(_is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT _is_multi_config AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type (default RelWithDebInfo)" FORCE)
    message(STATUS "No CMAKE_BUILD_TYPE set — defaulting to RelWithDebInfo (optimized).")
endif()

# Metal/Vulkan use a 0..1 clip-space depth range; make glm match engine-wide so projection
# matrices are consistent everywhere they're built (Camera::projectionMatrix, tests, ...).
add_compile_definitions(GLM_FORCE_DEPTH_ZERO_TO_ONE)

# Headless TRAINING build: only the ECS-free sim surface (core + physics + physics_env). Skips the
# graphics stack (RHI/Metal/Vulkan), GLFW, shaders, and the test/visual runners — so the Python
# binding links clean on a headless Linux box with no GPU/display SDK. Full dev build stays the default.
option(ENGINE_TRAINING_ONLY "Build only the headless training surface (core+physics+physics_env)" OFF)

# external dependencies (always needed by core/physics)
find_package(glm REQUIRED)
find_package(Threads REQUIRED)

# Graphics-only third-party deps (mesh/image loaders, windowing) — only the graphics module uses
# these, so a headless training build skips them (and their git submodules need not be initialized).
if(NOT ENGINE_TRAINING_ONLY)
    add_subdirectory(external/tinyobjloader)   # .obj mesh loader (graphics only)

    add_library(stb INTERFACE)                 # stb_image (graphics only)
    add_library(stb::stb ALIAS stb)
    target_include_directories(stb INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/external/stb
    )

    add_subdirectory(external/glfw)
    if(APPLE)
        message(STATUS "Apple platform detected. Configuring for Metal Backend.")
        enable_language(OBJC)     # keep .m (e.g. GLFW's) as Objective-C
        enable_language(OBJCXX)   # for the .mm CAMetalLayer/window shim
        find_library(METAL_FRAMEWORK Metal REQUIRED)
        find_library(QUARTZ_FRAMEWORK QuartzCore REQUIRED)
        find_library(FOUNDATION_FRAMEWORK Foundation REQUIRED)
        find_library(COCOA_FRAMEWORK Cocoa REQUIRED)
    else()
        message(STATUS "Non-Apple platform detected. Configuring for Vulkan Backend.")
        find_package(Vulkan REQUIRED)
    endif()
else()
    message(STATUS "ENGINE_TRAINING_ONLY=ON — headless build (core+physics+physics_env), skipping graphics/GLFW/shaders/tests.")
endif()

# engine modules
add_subdirectory(modules)

if(NOT ENGINE_TRAINING_ONLY)
    # shaders (compiled via slangc)
    add_subdirectory(src/shaders)

    # tests
    enable_testing()
    add_subdirectory(tst)
endif()

# Graphics-free aggregate for headless training / the Python binding — always available.
add_library(engine_training INTERFACE)
add_library(engine::training ALIAS engine_training)
target_link_libraries(engine_training INTERFACE
    engine::core
    engine::physics
    engine::physics_env
)

# Full engine meta target (everything, incl. graphics) — only in a full dev build.
if(NOT ENGINE_TRAINING_ONLY)
    add_library(engine INTERFACE)
    add_library(engine::engine ALIAS engine)
    target_link_libraries(engine INTERFACE
        engine::core
        engine::input
        engine::input_glfw
        engine::controls
        engine::ecs
        engine::rhi
        engine::render
        engine::scene
        engine::physics
        engine::physics_ecs
        engine::physics_env
    )
endif()
