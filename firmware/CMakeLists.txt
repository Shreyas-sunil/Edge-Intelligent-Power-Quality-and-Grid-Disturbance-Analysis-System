# =============================================================================
#  Edge-Intelligent Power Quality firmware — Raspberry Pi Pico 2 (RP2350)
#  Also builds for the original Pico (RP2040) with -DPICO_BOARD=pico.
#
#    cmake -S firmware -B build -G Ninja -DPICO_BOARD=pico2
#    cmake --build build
#    -> build/edge_pq.uf2
#
#  Options:
#    -DPQ_ENABLE_TFLM=OFF         DSP-only firmware (no TensorFlow Lite Micro)
#    -DPQ_TFLM_DIR=<path>         use a local pico-tflmicro checkout (offline)
#    -DPQ_DEFAULT_SIMULATION=0    boot on the hardware ADC instead of the simulator
#    -DPQ_USE_MCP4822=ON          drive DVR/APF references through an MCP4822 DAC
#    -DPQ_COPY_TO_RAM=ON          execute from SRAM (deterministic timing, RP2350)
#    -DPQ_UART_STDIO=ON           also route stdio to UART0 (debug)
# =============================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type (pico2 = RP2350, pico = RP2040)")

include(pico_sdk_import.cmake)

project(edge_pq C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
  message(FATAL_ERROR "Pico SDK >= 2.0.0 required (found ${PICO_SDK_VERSION_STRING})")
endif()

pico_sdk_init()

option(PQ_ENABLE_TFLM "Build with TensorFlow Lite Micro" ON)
option(PQ_USE_MCP4822 "Reference outputs via MCP4822 SPI DAC" OFF)
option(PQ_COPY_TO_RAM "Run the whole image from SRAM" OFF)
set(PQ_DEFAULT_SIMULATION 1 CACHE STRING "1 = simulated ADC at boot, 0 = hardware ADC")
set(PQ_TENSOR_ARENA_SIZE 4096 CACHE STRING "TFLM tensor arena bytes (measured use is reported by MODEL)")

include(${CMAKE_CURRENT_LIST_DIR}/cmake/pq_sources.cmake)

add_executable(edge_pq
  ${PQ_CORE_SOURCES}
  src/main.cpp
  src/platform/pico/platform_pico.cpp
  src/platform/pico/ads131m02_pico.cpp
  src/platform/pico/pico_io.cpp
  src/platform/pico/atomic_barrier.S)

target_include_directories(edge_pq PRIVATE ${PQ_SRC_DIR} ${PQ_CONFIG_DIR})
if(PICO_PLATFORM STREQUAL "rp2040")
  # 264 KB SRAM and no FPU: cap the FFT buffers at 1024 points (see docs/performance.md)
  target_compile_definitions(edge_pq PRIVATE PQ_FFT_MAX=1024)
endif()
target_compile_definitions(edge_pq PRIVATE
  PQ_DEFAULT_SIMULATION=${PQ_DEFAULT_SIMULATION}
  PQ_TENSOR_ARENA_SIZE=${PQ_TENSOR_ARENA_SIZE}
  PQ_USE_MCP4822=$<BOOL:${PQ_USE_MCP4822}>
  PICO_STACK_SIZE=0x1000                     # 4 KB core0 stack (all of SCRATCH_Y); core1 has its own 8 KB
  PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000      # never block long on a stalled host
  PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
  PICO_USE_MALLOC_MUTEX=1)
target_compile_options(edge_pq PRIVATE -Wall -Wextra -Wno-unused-parameter -O2)

target_link_libraries(edge_pq
  pico_stdlib
  pico_multicore
  pico_flash
  pico_stdio_usb
  hardware_dma
  hardware_pio
  hardware_pwm
  hardware_spi
  hardware_uart
  hardware_flash
  hardware_watchdog
  hardware_clocks)

if(PQ_ENABLE_TFLM)
  include(${CMAKE_CURRENT_LIST_DIR}/cmake/pq_tflm.cmake)
  target_compile_options(pq_tflm PRIVATE -O2)
  target_link_libraries(pq_tflm PUBLIC pico_stdlib)   # newlib/libc settings of the SDK
  target_link_libraries(edge_pq pq_tflm)
  target_compile_definitions(edge_pq PRIVATE PQ_HAVE_TFLM=1)
else()
  target_compile_definitions(edge_pq PRIVATE PQ_HAVE_TFLM=0)
endif()

pico_enable_stdio_usb(edge_pq 1)
pico_enable_stdio_uart(edge_pq 0)
if(PQ_COPY_TO_RAM)
  pico_set_binary_type(edge_pq copy_to_ram)
endif()
pico_set_program_name(edge_pq "edge_pq")
pico_set_program_description(edge_pq "Edge-intelligent power quality analyser")
pico_set_program_version(edge_pq "1.0.0")
pico_add_extra_outputs(edge_pq)   # .uf2 .bin .hex .dis .map

# Size report after every build (flash / RAM usage)
get_filename_component(_pq_cc_dir ${CMAKE_C_COMPILER} DIRECTORY)
find_program(PQ_SIZE_TOOL NAMES arm-none-eabi-size riscv32-unknown-elf-size HINTS ${_pq_cc_dir})
if(PQ_SIZE_TOOL)
  add_custom_command(TARGET edge_pq POST_BUILD
    COMMAND ${PQ_SIZE_TOOL} --format=berkeley $<TARGET_FILE:edge_pq>
    COMMENT "Firmware size (text = flash code+rodata, data+bss = static RAM)")
endif()
