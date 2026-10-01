// Pico implementation of the platform layer.
#include <malloc.h>

#include "common/platform.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/platform.h"
#include "pico/stdlib.h"

// Linker-script symbols (declared as arrays: they are addresses, not objects).
extern "C" {
extern uint8_t __data_start__[], __bss_end__[];
extern uint32_t __StackBottom[], __StackTop[];
extern uint8_t __flash_binary_start[], __flash_binary_end[];
}

namespace pq::plat {

uint64_t timeUs() { return time_us_64(); }
void sleepMs(uint32_t ms) { sleep_ms(ms); }
int coreNum() { return static_cast<int>(get_core_num()); }

namespace {
constexpr uint32_t kPaint = 0xA5A5A5A5u;
uint32_t* g_core1_stack = nullptr;
uint32_t g_core1_stack_words = 0;

uint32_t unusedWords(const volatile uint32_t* lo, uint32_t words) {
    uint32_t n = 0;
    while (n < words && lo[n] == kPaint) ++n;
    return n;
}
}  // namespace

// Called by main(): fill unused stack with a pattern to measure high-water marks.
void paintStacks(uint32_t* core1_stack, uint32_t core1_words) {
    volatile uint32_t* lo = __StackBottom;
    uint32_t sp;
    __asm volatile("mov %0, sp" : "=r"(sp));
    volatile uint32_t* hi = reinterpret_cast<volatile uint32_t*>(sp - 256);
    for (volatile uint32_t* p = lo; p < hi; ++p) *p = kPaint;
    g_core1_stack = core1_stack;
    g_core1_stack_words = core1_words;
    for (uint32_t k = 0; k < core1_words; ++k) core1_stack[k] = kPaint;
}

MemoryInfo memoryInfo() {
    MemoryInfo m{};
#if PICO_RP2350
    m.ram_total = 520u * 1024u;
#else
    m.ram_total = 264u * 1024u;
#endif
    m.ram_static = static_cast<uint32_t>(__bss_end__ - __data_start__);
    struct mallinfo mi = mallinfo();
    m.heap_used = static_cast<uint32_t>(mi.uordblks);
    m.flash_image = static_cast<uint32_t>(__flash_binary_end - __flash_binary_start);
    const uint32_t words0 = static_cast<uint32_t>(__StackTop - __StackBottom);
    m.stack_core0_free = 4u * unusedWords(__StackBottom, words0);
    if (g_core1_stack) m.stack_core1_free = 4u * unusedWords(g_core1_stack, g_core1_stack_words);
    return m;
}

void requestReboot() {
    watchdog_reboot(0, 0, 10);
    while (true) tight_loop_contents();
}

const char* platformName() {
#if PICO_RP2350
    return "rp2350";
#else
    return "rp2040";
#endif
}

uint32_t cpuHz() { return clock_get_hz(clk_sys); }

}  // namespace pq::plat

