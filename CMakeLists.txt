cmake_minimum_required(VERSION 3.15)

project(engine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# app
if (!APPLE)
    add_executable(main src/application.cpp)
else()
    # For macOS app bundle (optional) — create a .app so MoltenVK frameworks can be embedded
    add_executable(main MACOSX_BUNDLE src/application.cpp)
    set_target_properties(main PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_BUNDLE_NAME "EngineBundle"
        RESOURCE "${CMAKE_SOURCE_DIR}/resources"
    )
endif()

# potentially consolidate these packages ... 

# glm
find_package(glm REQUIRED)
target_link_libraries(main PRIVATE glm::glm)
target_include_directories(main PRIVATE ${GLM_INCLUDE_DIRS})

# glfw 
add_subdirectory(external/glfw)
include_directories(
    external/glfw/include
    ${GLFW_INCLUDE_DIRS}
)
target_link_libraries(main PRIVATE glfw)

# vulkan
find_package(Vulkan REQUIRED)
include_directories(
    ${Vulkan_INCLUDE_DIRS}
)
target_link_libraries(main PRIVATE Vulkan::Vulkan)

# tiny obj loader
add_subdirectory(external/tinyobjloader)
target_link_libraries(main PRIVATE tinyobjloader)

# stb
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/external/stb)
