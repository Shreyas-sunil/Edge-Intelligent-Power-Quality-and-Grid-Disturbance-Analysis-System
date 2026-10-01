# =============================================================================
#  Host (PC) build: unit + integration tests, the full-firmware simulator
#  (pq_sim), the ML dataset generator (pq_datagen) and kernel benchmarks
#  (pq_bench). Uses exactly the same sources as the Pico firmware.
#
#    cmake -S firmware/host -B build-host -DCMAKE_BUILD_TYPE=Release
#    cmake --build build-host -j
#    ctest --test-dir build-host --output-on-failure
# =============================================================================
cmake_minimum_required(VERSION 3.16)
project(edge_pq_host C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(PQ_ENABLE_TFLM "Build with TensorFlow Lite Micro" ON)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/pq_sources.cmake)

add_library(pq_core STATIC ${PQ_CORE_SOURCES} ${PQ_SRC_DIR}/platform/host/platform_host.cpp
            ${PQ_SRC_DIR}/platform/host/host_io.cpp)
target_include_directories(pq_core PUBLIC ${PQ_SRC_DIR} ${PQ_CONFIG_DIR})
if(MSVC)
  target_compile_options(pq_core PRIVATE /W3)
  target_compile_definitions(pq_core PUBLIC _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(pq_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

if(PQ_ENABLE_TFLM)
  include(${CMAKE_CURRENT_LIST_DIR}/../cmake/pq_tflm.cmake)
  target_link_libraries(pq_core PUBLIC pq_tflm)
  target_compile_definitions(pq_core PUBLIC PQ_HAVE_TFLM=1 PQ_TENSOR_ARENA_SIZE=8192)
else()
  target_compile_definitions(pq_core PUBLIC PQ_HAVE_TFLM=0)
endif()

find_package(Threads)
if(Threads_FOUND)
  target_link_libraries(pq_core PUBLIC Threads::Threads)
endif()

add_executable(pq_sim ${PQ_SRC_DIR}/platform/host/host_main.cpp)
target_link_libraries(pq_sim PRIVATE pq_core)

add_executable(pq_datagen ${CMAKE_CURRENT_LIST_DIR}/pq_datagen.cpp)
target_link_libraries(pq_datagen PRIVATE pq_core)

add_executable(pq_bench ${CMAKE_CURRENT_LIST_DIR}/pq_bench.cpp)
target_link_libraries(pq_bench PRIVATE pq_core)

# ----------------------------------------------------------------- tests
enable_testing()
file(GLOB PQ_TEST_SOURCES ${CMAKE_CURRENT_LIST_DIR}/../tests/test_*.cpp)
add_executable(pq_tests ${CMAKE_CURRENT_LIST_DIR}/../tests/test_main.cpp ${PQ_TEST_SOURCES})
target_include_directories(pq_tests PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../tests)
target_link_libraries(pq_tests PRIVATE pq_core)
foreach(t adc filter rms freq fft thd wavelet transient detector feature ml fusion protocol config control logger integration sim)
  add_test(NAME ${t} COMMAND pq_tests ${t})
endforeach()
