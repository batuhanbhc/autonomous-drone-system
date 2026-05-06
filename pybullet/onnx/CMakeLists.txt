cmake_minimum_required(VERSION 3.16)
project(marl_agent_onnx_runner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to an ONNX Runtime SDK directory containing include/ and lib/")
set(
  ONNXRUNTIME_INCLUDE_DIR
  ""
  CACHE PATH
  "Path to the directory that contains onnxruntime_cxx_api.h"
)
set(
  ONNXRUNTIME_LIB_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/venv/lib/python3.11/site-packages/onnxruntime/capi"
  CACHE PATH
  "Path to the directory that contains libonnxruntime.so"
)

if(ONNXRUNTIME_ROOT)
  list(PREPEND _ort_include_hints "${ONNXRUNTIME_ROOT}/include")
  list(PREPEND _ort_lib_hints "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64")
endif()

if(ONNXRUNTIME_INCLUDE_DIR)
  list(PREPEND _ort_include_hints "${ONNXRUNTIME_INCLUDE_DIR}")
endif()

if(ONNXRUNTIME_LIB_DIR)
  list(PREPEND _ort_lib_hints "${ONNXRUNTIME_LIB_DIR}")
endif()

find_path(
  ORT_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${_ort_include_hints}
)

find_library(
  ORT_LIBRARY
  NAMES onnxruntime
  HINTS ${_ort_lib_hints}
)

if(NOT ORT_INCLUDE_DIR)
  message(FATAL_ERROR
    "Could not find onnxruntime_cxx_api.h.\n"
    "Install the ONNX Runtime C/C++ headers and configure either:\n"
    "  -DONNXRUNTIME_ROOT=/path/to/onnxruntime\n"
    "or\n"
    "  -DONNXRUNTIME_INCLUDE_DIR=/path/to/include"
  )
endif()

if(NOT ORT_LIBRARY)
  message(FATAL_ERROR
    "Could not find libonnxruntime.so.\n"
    "Configure either:\n"
    "  -DONNXRUNTIME_ROOT=/path/to/onnxruntime\n"
    "or\n"
    "  -DONNXRUNTIME_LIB_DIR=/path/to/lib"
  )
endif()

add_executable(run_random_inference run_random_inference.cpp)
target_include_directories(run_random_inference PRIVATE "${ORT_INCLUDE_DIR}")
target_link_libraries(run_random_inference PRIVATE "${ORT_LIBRARY}")

add_executable(benchmark_inference benchmark_inference.cpp)
target_include_directories(benchmark_inference PRIVATE "${ORT_INCLUDE_DIR}")
target_link_libraries(benchmark_inference PRIVATE "${ORT_LIBRARY}")

get_filename_component(ORT_LIBRARY_DIR "${ORT_LIBRARY}" DIRECTORY)
set_target_properties(
  run_random_inference
  PROPERTIES
    BUILD_RPATH "${ORT_LIBRARY_DIR}"
    INSTALL_RPATH "${ORT_LIBRARY_DIR}"
)

set_target_properties(
  benchmark_inference
  PROPERTIES
    BUILD_RPATH "${ORT_LIBRARY_DIR}"
    INSTALL_RPATH "${ORT_LIBRARY_DIR}"
)

message(STATUS "Using ONNX Runtime include dir: ${ORT_INCLUDE_DIR}")
message(STATUS "Using ONNX Runtime library: ${ORT_LIBRARY}")
