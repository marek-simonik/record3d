project(lzfse)
cmake_minimum_required(VERSION 3.12)

include(CheckCCompilerFlag)

# Compiler flags
function(lzfse_add_compiler_flags target)
  set (flags ${ARGV})
  list (REMOVE_AT flags 0)

  foreach (FLAG ${flags})
    if(CMAKE_C_COMPILER_ID STREQUAL GNU)
      # Because https://gcc.gnu.org/wiki/FAQ#wnowarning
      string(REGEX REPLACE "\\-Wno\\-(.+)" "-W\\1" flag_to_test "${FLAG}")
    else()
      set (flag_to_test ${FLAG})
    endif()

    string(REGEX REPLACE "[^a-zA-Z0-9]+" "_" test_name "CFLAG_${flag_to_test}")

    check_c_compiler_flag("${flag_to_test}" "${test_name}")
    if(${${test_name}})
      set_property(TARGET "${target}" APPEND_STRING PROPERTY COMPILE_FLAGS " ${FLAG}")
    endif()
  endforeach()
endfunction()

add_library(lzfse STATIC
  src/lzfse_decode.c
  src/lzfse_decode_base.c
  src/lzfse_encode.c
  src/lzfse_encode_base.c
  src/lzfse_fse.c
  src/lzvn_decode_base.c
  src/lzvn_encode_base.c)
lzfse_add_compiler_flags(lzfse -Wall -Wno-unknown-pragmas -Wno-unused-variable)

set_target_properties(lzfse PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  C_VISIBILITY_PRESET hidden
  INTERPROCEDURAL_OPTIMIZATION TRUE)