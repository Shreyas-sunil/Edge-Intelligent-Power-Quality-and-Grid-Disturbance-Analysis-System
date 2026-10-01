// Thin platform abstraction. Everything above this layer (DSP, detection,
// features, ML, fusion, protocol, logger) is platform independent and is
// compiled unchanged for the Pico firmware AND for the host test/sim build.
//
// Implementations: src/platform/pico/platform_pico.cpp
//                  src/platform/host/platform_host.cpp
#pragma once
#include <cstddef>
#include <cstdint>

namespace pq::plat {

uint64_t timeUs();            // monotonic microseconds since boot
inline uint32_t timeUs32() { return static_cast<uint32_t>(timeUs()); }
void     sleepMs(uint32_t ms);
int      coreNum();           // 0 or 1 (host: always 0)

// Memory accounting (best effort; host returns 0 where meaningless).
struct MemoryInfo {
    uint32_t ram_total;       // bytes of SRAM on the part
    uint32_t ram_static;      // .data + .bss
    uint32_t heap_used;       // malloc'd bytes (should stay ~constant after init)
    uint32_t flash_image;     // bytes of the firmware image in flash
    uint32_t stack_core0_free;
    uint32_t stack_core1_free;
};
MemoryInfo memoryInfo();

void        requestReboot();      // watchdog-based reboot
const char* platformName();       // "rp2350", "rp2040", "host"
uint32_t    cpuHz();

}  // namespace pq::plat
