cmake_minimum_required(VERSION 3.15)

project(engine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# app
# add_executable(main main.cpp)

# For macOS app bundle (optional) — create a .app so MoltenVK frameworks can be embedded
add_executable(main MACOSX_BUNDLE main.cpp)
set_target_properties(main PROPERTIES
  MACOSX_BUNDLE TRUE
  MACOSX_BUNDLE_BUNDLE_NAME "EngineBundle"
  RESOURCE "${CMAKE_SOURCE_DIR}/resources"
)

# glm
find_package(glm REQUIRED)
include_directories(
    ${GLM_INCLUDE_DIRS}
)
if (APPLE)
    target_link_libraries(main PRIVATE glm::glm)
else()
    target_link_libraries(main PRIVATE glm::glm ${Vulkan_LIBRARIES})
endif()

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
