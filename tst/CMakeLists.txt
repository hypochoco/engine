add_executable(tst test.cpp)
target_link_libraries(tst PRIVATE engine::core)

# RHI backend tests. Metal is the only implemented backend so far, so build these on Apple
# only; they become cross-backend once Vulkan is ported behind the RHI.
if(APPLE)
    add_executable(rhi_smoke rhi_smoke.cpp)
    target_link_libraries(rhi_smoke PRIVATE engine::graphics)

    add_executable(triangle_offscreen triangle_offscreen.cpp)
    target_link_libraries(triangle_offscreen PRIVATE engine::graphics)
    add_dependencies(triangle_offscreen shaders)
    target_compile_definitions(triangle_offscreen PRIVATE
        ENGINE_SHADER_DIR="${ENGINE_SHADER_DIR}")

    add_executable(mesh_offscreen mesh_offscreen.cpp)
    target_link_libraries(mesh_offscreen PRIVATE engine::graphics)
    add_dependencies(mesh_offscreen shaders)
    target_compile_definitions(mesh_offscreen PRIVATE
        ENGINE_SHADER_DIR="${ENGINE_SHADER_DIR}")

    add_executable(visual_window visual_window.cpp)
    target_link_libraries(visual_window PRIVATE engine::graphics)
    add_dependencies(visual_window shaders)
    target_compile_definitions(visual_window PRIVATE
        ENGINE_SHADER_DIR="${ENGINE_SHADER_DIR}")
endif()
