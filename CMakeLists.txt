cmake_minimum_required(VERSION 3.15)

project(engine)

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

add_executable(main main.cpp)

if (APPLE)
    target_link_libraries(main PRIVATE glfw glm::glm)
else()
    target_link_libraries(main PRIVATE glfw glm::glm ${Vulkan_LIBRARIES})
endif()

if (APPLE)
    find_library(MOLTENVK MoltenVK HINTS /usr/local/lib REQUIRED)
    target_link_libraries(main PRIVATE ${MOLTENVK})

    target_link_libraries(main
        PRIVATE
        "-framework Metal"
        "-framework QuartzCore"
        "-framework IOSurface"
    )

    include_directories(/usr/local/include/MoltenVK)
endif()