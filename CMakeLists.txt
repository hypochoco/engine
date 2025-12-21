
cmake_minimum_required(VERSION 3.25)
project(engine VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# external dependencies
find_package(glm REQUIRED)
find_package(Vulkan REQUIRED)
add_subdirectory(external/glfw)
add_subdirectory(external/tinyobjloader)

add_library(stb INTERFACE)
add_library(stb::stb ALIAS stb)
target_include_directories(stb INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/external/stb
)

# engine modules
add_subdirectory(graphics)
add_subdirectory(physics)

# engine meta target
add_library(engine INTERFACE)
add_library(engine::engine ALIAS engine)

target_link_libraries(engine INTERFACE
    engine::graphics
    engine::physics
)
