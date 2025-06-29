
cmake_minimum_required(VERSION 3.15)

project(game_engine)

set(CMAKE_CXX_STANDARD 17)

find_package(glm REQUIRED)
find_package(Vulkan REQUIRED)
add_subdirectory(external/glfw)
include_directories(
    external/glfw/include
    ${GLFW_INCLUDE_DIRS}
    ${Vulkan_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
)
if(APPLE) # moltenvk for mac
    find_library(MOLTENVK MoltenVK REQUIRED)
    include_directories(/usr/local/include/MoltenVK)
endif()

add_executable(main main.cpp)

target_link_libraries(main ${Vulkan_LIBRARIES} glfw glm::glm)
if (APPLE)
    target_link_libraries(main ${MOLTENVK})
endif()
