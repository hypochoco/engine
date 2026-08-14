# EngineShaders.cmake
#
# Reusable helper to compile Slang shaders to the active backend's format (.metallib on Apple,
# .spv elsewhere) with slangc. Used by the engine's own shaders and by consumers (games/tools) that
# build shader variants against the engine's shader contract (see shaders/engine_mesh.slang) — so a
# consumer never restates the slangc invocation or backend selection.
#
# Usage:
#   include(EngineShaders)                       # once CMAKE_MODULE_PATH includes engine/cmake
#   engine_compile_shaders(
#       TARGET       my_game_shaders             # custom target (added to ALL)
#       OUT_DIR      "${CMAKE_BINARY_DIR}/shaders"
#       SOURCES      foliage.slang terrain.slang # shaders WITH entry points
#       INCLUDE_DIRS "${ENGINE_SHADER_INCLUDE_DIR}"   # for #include "engine_mesh.slang"
#       DEPENDS      "${ENGINE_SHADER_INCLUDE_DIR}/engine_mesh.slang")  # rebuild when includes change
#
# slangc is located via the SLANGC cache var if set, else find_program (NAMES slangc) with hints
# from ENGINE_SLANG_DIR (the engine's vendored external/slang).

if(NOT DEFINED SLANGC OR NOT SLANGC)
    find_program(SLANGC
        NAMES slangc
        HINTS "${ENGINE_SLANG_DIR}/bin" "$ENV{ENGINE_SLANG_DIR}/bin")
endif()

function(engine_compile_shaders)
    cmake_parse_arguments(ES "" "TARGET;OUT_DIR" "SOURCES;INCLUDE_DIRS;DEPENDS" ${ARGN})

    if(NOT SLANGC)
        message(FATAL_ERROR
            "engine_compile_shaders: slangc not found. Set -DSLANGC=/path/to/slangc or "
            "-DENGINE_SLANG_DIR=/path/to/slang, or run engine/src/tools/get_slang.sh.")
    endif()
    if(NOT ES_TARGET OR NOT ES_OUT_DIR OR NOT ES_SOURCES)
        message(FATAL_ERROR "engine_compile_shaders: TARGET, OUT_DIR and SOURCES are required.")
    endif()

    if(APPLE)
        set(_target_flag "metallib")
        set(_ext "metallib")
    else()
        set(_target_flag "spirv")
        set(_ext "spv")
    endif()

    file(MAKE_DIRECTORY "${ES_OUT_DIR}")

    set(_inc_flags "")
    foreach(_dir ${ES_INCLUDE_DIRS})
        list(APPEND _inc_flags -I "${_dir}")
    endforeach()

    set(_outputs "")
    foreach(_src ${ES_SOURCES})
        get_filename_component(_name "${_src}" NAME_WE)
        get_filename_component(_abs  "${_src}" ABSOLUTE)
        set(_out "${ES_OUT_DIR}/${_name}.${_ext}")
        add_custom_command(
            OUTPUT "${_out}"
            COMMAND "${SLANGC}" "${_abs}" -target ${_target_flag} -matrix-layout-column-major
                    ${_inc_flags} -o "${_out}"
            MAIN_DEPENDENCY "${_abs}"
            DEPENDS ${ES_DEPENDS}
            COMMENT "slangc ${_name} -> ${_name}.${_ext} (${_target_flag})"
            VERBATIM)
        list(APPEND _outputs "${_out}")
    endforeach()

    add_custom_target(${ES_TARGET} ALL DEPENDS ${_outputs})
endfunction()
