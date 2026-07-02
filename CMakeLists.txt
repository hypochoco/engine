
cmake_minimum_required(VERSION 3.25)
project(engine VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# external dependencies
find_package(glm REQUIRED)
add_subdirectory(external/glfw)
add_subdirectory(external/tinyobjloader)

add_library(stb INTERFACE)
add_library(stb::stb ALIAS stb)
target_include_directories(stb INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/external/stb
)

if(APPLE)
    message(STATUS "Apple platform detected. Configuring for Metal Backend.")
    find_library(METAL_FRAMEWORK Metal REQUIRED)
    find_library(QUARTZ_FRAMEWORK QuartzCore REQUIRED)
else()
    message(STATUS "Non-Apple platform detected. Configuring for Vulkan Backend.")
    find_package(Vulkan REQUIRED)
endif()

# engine modules
add_subdirectory(core)
add_subdirectory(graphics)
add_subdirectory(physics)

# tests
add_subdirectory(tst)

# engine meta target
add_library(engine INTERFACE)
add_library(engine::engine ALIAS engine)

target_link_libraries(engine INTERFACE
    engine::core
    engine::graphics
    engine::physics
)
