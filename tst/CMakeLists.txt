# ---------------------------------------------------------------------------------------------
# Test architecture — organized by module (core/ecs/physics/graphics) then category:
#   <module>/unit/*.cpp         → fast, isolated pass/fail        ┐
#   <module>/integration/*.cpp  → cross-subsystem pass/fail       ┘ → `tests` (CTest per module)
#   <module>/benchmark/*.cpp    → perf, prints timings              → `benchmarks` (Release)
#   <module>/visual/*.cpp       → interactive windowed demos        → `visuals` (dispatch by name)
#
# Each file self-registers via TST_CASE(module, category, name) — drop a file in the right
# folder and it's picked up (CONFIGURE_DEPENDS re-globs), no list to edit. Graphics suites and
# all visuals require the Metal backend, so they're Apple-only for now.
# ---------------------------------------------------------------------------------------------

set(HARNESS harness/harness.cpp harness/runner_main.cpp)

file(GLOB_RECURSE UNIT_SRC  CONFIGURE_DEPENDS
    core/unit/*.cpp ecs/unit/*.cpp physics/unit/*.cpp input/unit/*.cpp controls/unit/*.cpp physics_env/unit/*.cpp)
file(GLOB_RECURSE INTEG_SRC CONFIGURE_DEPENDS
    core/integration/*.cpp ecs/integration/*.cpp physics/integration/*.cpp physics_env/integration/*.cpp)
file(GLOB_RECURSE BENCH_SRC CONFIGURE_DEPENDS
    core/benchmark/*.cpp ecs/benchmark/*.cpp physics/benchmark/*.cpp physics_env/benchmark/*.cpp)
if(APPLE)
    file(GLOB_RECURSE GFX_TEST_SRC CONFIGURE_DEPENDS
        graphics/unit/*.cpp graphics/integration/*.cpp)
    file(GLOB_RECURSE VISUAL_SRC   CONFIGURE_DEPENDS
        physics/visual/*.cpp graphics/visual/*.cpp)
endif()

# --- tests: unit + integration (headless pass/fail) ---
add_executable(tests ${HARNESS} ${UNIT_SRC} ${INTEG_SRC} ${GFX_TEST_SRC})
target_include_directories(tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(tests PRIVATE engine::physics_ecs engine::physics_env engine::input engine::controls)
if(APPLE)
    target_link_libraries(tests PRIVATE engine::scene)
    add_dependencies(tests shaders)
    target_compile_definitions(tests PRIVATE ENGINE_SHADER_DIR="${ENGINE_SHADER_DIR}")
endif()

foreach(module core ecs physics graphics input controls physics_env)
    add_test(NAME ${module} COMMAND tests --module ${module})
endforeach()

# --- benchmarks: perf suites (run in Release; not pass/fail) ---
add_executable(benchmarks ${HARNESS} ${BENCH_SRC})
target_include_directories(benchmarks PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(benchmarks PRIVATE engine::physics_ecs engine::physics_env)

# --- visuals: interactive windowed demos (Apple; run one by name, e.g. `visuals rolling`) ---
if(APPLE)
    add_executable(visuals ${HARNESS} ${VISUAL_SRC})
    target_include_directories(visuals PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(visuals PRIVATE engine::scene engine::physics_ecs engine::input engine::input_glfw engine::controls)
    add_dependencies(visuals shaders)
    target_compile_definitions(visuals PRIVATE ENGINE_SHADER_DIR="${ENGINE_SHADER_DIR}")
endif()
