


# cmake_minimum_required(VERSION 3.15)

# project(game_engine)

# set(CMAKE_CXX_STANDARD 17)

# # Find Vulkan and GLFW
# find_package(Vulkan REQUIRED)
# find_package(glfw3 REQUIRED)

# add_subdirectory(external/glfw)
# include_directories(
#     external/glfw/include
# )

# # Include directories
# include_directories(${Vulkan_INCLUDE_DIRS})
# include_directories(${GLFW_INCLUDE_DIRS})

# # On macOS, you may need to link MoltenVK
# if(APPLE)
#     # Find MoltenVK installed by Homebrew
#     find_library(MOLTENVK MoltenVK REQUIRED)
#     include_directories(/usr/local/include/MoltenVK)
# endif()

# # Add your source file
# add_executable(main main.cpp)

# # Link Vulkan and GLFW libraries
# target_link_libraries(main ${Vulkan_LIBRARIES} glfw ${MOLTENVK})












cmake_minimum_required(VERSION 3.15)

project(game_engine)

set(CMAKE_CXX_STANDARD 17)

# Find Vulkan and GLFW
find_package(Vulkan REQUIRED)
# find_package(glfw3 REQUIRED)

add_subdirectory(external/glfw)
include_directories(
    external/glfw/include
)

# Include directories
include_directories(${Vulkan_INCLUDE_DIRS})
include_directories(${GLFW_INCLUDE_DIRS})

# On macOS, you may need to link MoltenVK
if(APPLE)
    # Find MoltenVK installed by Homebrew
    find_library(MOLTENVK MoltenVK REQUIRED)
    include_directories(/usr/local/include/MoltenVK)
endif()

# Add your source file
add_executable(main main.cpp)

# Link Vulkan and GLFW libraries
target_link_libraries(main ${Vulkan_LIBRARIES} glfw ${MOLTENVK})














# cmake_minimum_required(VERSION 3.15)

# project(game_engine)

# find_package(OpenGL REQUIRED)

# add_subdirectory(external/glfw)
# include_directories(
#     ${OPENGL_INCLUDE_DIRS}
#     external/glfw/include
# )

# add_executable(main main.cpp)

# target_link_libraries(main ${OPENGL_LIBRARIES} glfw)






