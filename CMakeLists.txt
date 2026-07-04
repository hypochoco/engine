
cmake_minimum_required(VERSION 3.25)
project(engine VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Metal/Vulkan use a 0..1 clip-space depth range; make glm match engine-wide so projection
# matrices are consistent everywhere they're built (Camera::projectionMatrix, tests, ...).
add_compile_definitions(GLM_FORCE_DEPTH_ZERO_TO_ONE)

# external dependencies
find_package(glm REQUIRED)
find_package(Threads REQUIRED)
add_subdirectory(external/glfw)
add_subdirectory(external/tinyobjloader)

add_library(stb INTERFACE)
add_library(stb::stb ALIAS stb)
target_include_directories(stb INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/external/stb
)

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

# engine modules
add_subdirectory(modules)

# shaders (compiled via slangc)
add_subdirectory(shaders)

# tests
enable_testing()
add_subdirectory(tst)

# engine meta target
add_library(engine INTERFACE)
add_library(engine::engine ALIAS engine)

target_link_libraries(engine INTERFACE
    engine::core
    engine::input
    engine::input_glfw
    engine::controls
    engine::ecs
    engine::graphics
    engine::scene
    engine::physics
    engine::physics_ecs
)
