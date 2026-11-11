# PhysicalAI 보조 플러그인 모음(physai_plugins.dll)
# - 본 트리 안에서는 add_subdirectory(plugin/physai) 로 mujoco 타깃에 링크된다.
# - 단독 빌드(mujoco.dll 재빌드 없이): cmake -S plugin/physai -B build -DMUJOCO_ROOT=<공식 릴리스 폴더>
#   (MUJOCO_ROOT 에는 include/ 와 lib/mujoco.lib 가 있어야 한다)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.16)
  project(physai_plugins CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS OFF)
  if(NOT MUJOCO_ROOT)
    set(MUJOCO_ROOT "$ENV{MUJOCO_ROOT}")
  endif()
  if(NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
    message(FATAL_ERROR "MUJOCO_ROOT 가 올바르지 않음: ${MUJOCO_ROOT}")
  endif()
  add_library(mujoco SHARED IMPORTED)
  set_target_properties(mujoco PROPERTIES
    IMPORTED_IMPLIB "${MUJOCO_ROOT}/lib/mujoco.lib"
    IMPORTED_LOCATION "${MUJOCO_ROOT}/bin/mujoco.dll"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include")
  set(PHYSAI_OUT_DIR "${CMAKE_BINARY_DIR}")
else()
  set(PHYSAI_OUT_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/mujoco_plugin")
endif()

add_library(physai_plugins SHARED
  register.cc
  drivetrain.cc
  drivetrain.h
  winch.cc
  winch.h
)
set_target_properties(physai_plugins PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${PHYSAI_OUT_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${PHYSAI_OUT_DIR}")
foreach(cfg RELEASE DEBUG RELWITHDEBINFO MINSIZEREL)
  set_target_properties(physai_plugins PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_${cfg} "${PHYSAI_OUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_${cfg} "${PHYSAI_OUT_DIR}")
endforeach()
target_link_libraries(physai_plugins PRIVATE mujoco)
if(MSVC)
  target_compile_options(physai_plugins PRIVATE /utf-8)
endif()
