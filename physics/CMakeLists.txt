
# physics 
add_library(engine_physics STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/physics/physics.cpp
)

add_library(engine::physics ALIAS engine_physics)

target_include_directories(engine_physics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
