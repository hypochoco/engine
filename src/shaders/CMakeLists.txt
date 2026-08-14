# shaders
#
# Compiles the engine's Slang shaders via the reusable engine_compile_shaders() helper
# (cmake/EngineShaders.cmake) to the active backend's format (.metallib on Apple, .spv elsewhere).
# Consumers reuse the SAME helper to build their own shader variants against the shared contract in
# engine_mesh.slang (exposed via ENGINE_SHADER_INCLUDE_DIR).

# Anchor to the ENGINE's own directory (src/shaders → engine root/external/slang), NOT
# CMAKE_SOURCE_DIR — the latter is the CONSUMER's root when the engine is add_subdirectory'd, which
# would look for slang in the wrong place.
set(ENGINE_SLANG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../external/slang" CACHE PATH "Vendored Slang toolchain")
include(EngineShaders)

set(SHADER_OUT_DIR "${CMAKE_BINARY_DIR}/shaders")

# Shader sources WITH entry points (engine_mesh.slang is an include-only contract, not listed here).
set(SHADER_SOURCES
    triangle.slang
    mesh.slang
    compute_smoke.slang
    cluster.slang
    tonemap.slang
    shadow.slang
    sky.slang
    fxaa.slang
)
list(TRANSFORM SHADER_SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

engine_compile_shaders(
    TARGET       shaders
    OUT_DIR      "${SHADER_OUT_DIR}"
    SOURCES      ${SHADER_SOURCES}
    INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS      "${CMAKE_CURRENT_SOURCE_DIR}/engine_mesh.slang"
)

# Expose the compiled-output dir and the shader INCLUDE dir (the shared Slang contract) to consumers.
set(ENGINE_SHADER_DIR "${SHADER_OUT_DIR}" CACHE INTERNAL "Compiled shader output directory")
set(ENGINE_SHADER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "Engine shader include dir (engine_mesh.slang)")
