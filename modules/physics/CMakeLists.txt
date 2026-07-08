# physics
#
# Phase-0 core: ECS-free, backend-agnostic (shapes, primitive collision, rigid-body state,
# integration kernels). Depends on engine::core only (glm). No ecs, no graphics.
# Plan: notes/investigations/2026-07-03-physics-plan.md.

file(GLOB_RECURSE PHYSICS_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics/*.hpp"
)

file(GLOB_RECURSE PHYSICS_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics/*.cpp"
)

# CUDA backend sources — compiled only when ENGINE_CUDA is ON (NVIDIA only). Kept out of the default
# build so the CPU/Mac path is byte-for-byte unchanged. At scaffolding time no .cu exists yet, so the
# glob is simply empty; the kernel lands here in a later phase (src/physics/cuda/*.cu).
set(PHYSICS_CUDA_SOURCE "")
if(ENGINE_CUDA)
    file(GLOB_RECURSE PHYSICS_CUDA_SOURCE
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics/*.cu"
    )
endif()

add_library(engine_physics STATIC
    ${PHYSICS_HEADERS}
    ${PHYSICS_SOURCE}
    ${PHYSICS_CUDA_SOURCE}
)
add_library(engine::physics ALIAS engine_physics)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../include/engine/physics/" PREFIX "Header Files" FILES ${PHYSICS_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../../src/physics/" PREFIX "Source Files" FILES ${PHYSICS_SOURCE})

target_include_directories(engine_physics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(engine_physics
    PUBLIC
        engine::core
)

# CUDA backend wiring (guarded; NVIDIA only). PUBLIC ENGINE_CUDA so downstream (physics_env /
# CudaVecEnv) can guard the device path on the same macro. Separable compilation + cudart link are
# no-ops until the first .cu lands, but keep the target ready for it.
if(ENGINE_CUDA)
    target_compile_definitions(engine_physics PUBLIC ENGINE_CUDA)
    set_target_properties(engine_physics PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_RESOLVE_DEVICE_SYMBOLS ON)   # device-link within the lib so non-CUDA consumers link clean
    # std::array (constexpr operator[]) is used on the device hot path — allow calling constexpr host
    # functions from device code. Applies to .cu TUs only.
    target_compile_options(engine_physics PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
    target_link_libraries(engine_physics PUBLIC CUDA::cudart)
endif()
