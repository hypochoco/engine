# shaders
#
# Compiles single-source .slang shaders with slangc to the active backend's format:
# a Metal library (.metallib) on Apple, SPIR-V (.spv) elsewhere. Outputs land in
# ${CMAKE_BINARY_DIR}/shaders; consumers get that path via the ENGINE_SHADER_DIR define.

find_program(SLANGC
    NAMES slangc
    HINTS "${CMAKE_SOURCE_DIR}/external/slang/bin"
)
if(NOT SLANGC)
    message(FATAL_ERROR
        "slangc not found. Run src/tools/get_slang.sh to download the Slang toolchain into external/slang/.")
endif()

set(SHADER_OUT_DIR "${CMAKE_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "${SHADER_OUT_DIR}")

# List of shader sources (add new .slang files here).
set(SHADER_SOURCES
    triangle.slang
    mesh.slang
    compute_smoke.slang
    cluster.slang
    tonemap.slang
    shadow.slang
    sky.slang
)

set(SHADER_OUTPUTS)
foreach(src ${SHADER_SOURCES})
    get_filename_component(name "${src}" NAME_WE)
    set(in "${CMAKE_CURRENT_SOURCE_DIR}/${src}")
    if(APPLE)
        set(out "${SHADER_OUT_DIR}/${name}.metallib")
        set(target_flag "metallib")
    else()
        set(out "${SHADER_OUT_DIR}/${name}.spv")
        set(target_flag "spirv")
    endif()
    add_custom_command(
        OUTPUT "${out}"
        COMMAND "${SLANGC}" "${in}" -target ${target_flag} -matrix-layout-column-major -o "${out}"
        MAIN_DEPENDENCY "${in}"
        COMMENT "slangc ${src} -> ${name} (${target_flag})"
        VERBATIM
    )
    list(APPEND SHADER_OUTPUTS "${out}")
endforeach()

add_custom_target(shaders ALL DEPENDS ${SHADER_OUTPUTS})

# Expose the output directory to consumers.
set(ENGINE_SHADER_DIR "${SHADER_OUT_DIR}" CACHE INTERNAL "Compiled shader output directory")
