# engine modules
#
# Each subdirectory is a static library target (engine::<module>). Order matters: a module must
# be added before others that link it. Source lives under ../src and ../include; these dirs hold
# only the per-module build files.

add_subdirectory(core)
add_subdirectory(physics)
add_subdirectory(physics_env)

# Graphics / ECS / input modules — not needed for a headless training build.
if(NOT ENGINE_TRAINING_ONLY)
    add_subdirectory(input)
    add_subdirectory(input_glfw)
    add_subdirectory(ecs)
    add_subdirectory(controls)
    add_subdirectory(graphics)
    add_subdirectory(scene)
    add_subdirectory(pathtracer)
    add_subdirectory(physics_ecs)
endif()
