# =============================================================================
#  TensorFlow Lite Micro (Raspberry Pi's pico-tflmicro port, which bundles the
#  CMSIS-NN optimised kernels) compiled as a static library `pq_tflm`.
#
#  Source:  -DPQ_TFLM_DIR=<path to a pico-tflmicro checkout>   (offline builds)
#           otherwise fetched from GitHub at a pinned commit.
#  The same file is used by the Pico build and by the host test build.
# =============================================================================
set(PQ_TFLM_GIT_TAG dfea53b977f131dc50d7060568ad4fd355d4d71e CACHE STRING "pico-tflmicro commit")

if(NOT PQ_TFLM_DIR)
  include(FetchContent)
  FetchContent_Declare(pico_tflmicro
    GIT_REPOSITORY https://github.com/raspberrypi/pico-tflmicro.git
    GIT_TAG        ${PQ_TFLM_GIT_TAG}
    SOURCE_SUBDIR  _pq_do_not_add_subdirectory)   # only download, do not add_subdirectory()
  FetchContent_MakeAvailable(pico_tflmicro)
  set(PQ_TFLM_DIR ${pico_tflmicro_SOURCE_DIR})
endif()

if(NOT EXISTS ${PQ_TFLM_DIR}/src/tensorflow/lite/micro/micro_interpreter.h)
  message(FATAL_ERROR "PQ_TFLM_DIR=${PQ_TFLM_DIR} is not a pico-tflmicro checkout")
endif()
message(STATUS "TensorFlow Lite Micro: ${PQ_TFLM_DIR}")

file(GLOB_RECURSE _tflm_src
  ${PQ_TFLM_DIR}/src/tensorflow/*.cc
  ${PQ_TFLM_DIR}/src/tensorflow/*.cpp
  ${PQ_TFLM_DIR}/src/tensorflow/*.c
  ${PQ_TFLM_DIR}/src/third_party/cmsis_nn/*.c)
list(FILTER _tflm_src EXCLUDE REGEX "_test\\.(cc|cpp|c)$")
list(FILTER _tflm_src EXCLUDE REGEX "/testing/|/benchmarks/|/examples/")
# Platform glue is provided by src/tinyml/tflm_platform.cpp
list(FILTER _tflm_src EXCLUDE REGEX "micro/system_setup\\.cpp$|micro/micro_time\\.cpp$")

# The pico-tflmicro port unconditionally enables an "experimental dual core"
# matmul (TF_LITE_PICO_MULTICORE) that pushes work to core1 via the multicore
# FIFO. Core1 is our hard real-time core, so that must never happen: compile a
# patched copy with the feature removed (it is also Pico-only and would not
# build on the host).
set(_mm ${PQ_TFLM_DIR}/src/third_party/cmsis_nn/Source/NNSupportFunctions/arm_nn_mat_mult_nt_t_s8.c)
file(READ ${_mm} _mm_txt)
string(REPLACE "#define TF_LITE_PICO_MULTICORE" "/* TF_LITE_PICO_MULTICORE disabled by edge-pq (core1 is reserved) */" _mm_txt "${_mm_txt}")
set(_mm_patched ${CMAKE_BINARY_DIR}/tflm_patched/arm_nn_mat_mult_nt_t_s8.c)
file(WRITE ${_mm_patched} "${_mm_txt}")
list(REMOVE_ITEM _tflm_src ${_mm})
list(APPEND _tflm_src ${_mm_patched})

add_library(pq_tflm STATIC ${_tflm_src} ${CMAKE_CURRENT_LIST_DIR}/../src/tinyml/tflm_platform.cpp)
target_include_directories(pq_tflm SYSTEM PUBLIC
  ${PQ_TFLM_DIR}/src
  ${PQ_TFLM_DIR}/src/third_party/ruy
  ${PQ_TFLM_DIR}/src/third_party/gemmlowp
  ${PQ_TFLM_DIR}/src/third_party/flatbuffers/include
  ${PQ_TFLM_DIR}/src/third_party/cmsis/CMSIS/Core/Include
  ${PQ_TFLM_DIR}/src/third_party/cmsis_nn
  ${PQ_TFLM_DIR}/src/third_party/cmsis_nn/Include)
target_compile_definitions(pq_tflm PUBLIC
  TF_LITE_STATIC_MEMORY=1
  ARDUINO=1                  # pico-tflmicro convention: strips desktop-only code paths
  TF_LITE_DISABLE_X86_NEON=1
  CMSIS_NN=1)
set_target_properties(pq_tflm PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(pq_tflm PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti -fno-exceptions -fno-threadsafe-statics>
  -w)   # third-party code: silence warnings
