// =============================================================================
//  Edge-Intelligent Power Quality & Grid Disturbance Analysis — Pico firmware
//
//  Core 1 (real time, never blocks):  ADC (DMA) -> calibration -> DSP ->
//          PQ detection -> features -> TinyML -> fusion -> fast outputs
//  Core 0 (supervisor):  USB/UART protocol, event log, metrics streaming,
//          alarms, calibration procedures, flash config, benchmarks, watchdog
//
//  Everything above the platform layer is the same code that the host tests
//  and the PC simulator (pq_sim) run.
// =============================================================================
#include <cstdio>

#include "app/realtime_task.h"
#include "app/supervisor.h"
#include "common/platform.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "platform/pico/ads131m02_pico.h"
#include "platform/pico/pico_io.h"
#include "tinyml/classifier.h"

namespace pq::plat {
void paintStacks(uint32_t* core1_stack, uint32_t core1_words);
}

using namespace pq;

// clk_sys must be an integer multiple of the ADS131M02 CLKIN (5.12 MHz) so the
// sample clock is exact: RP2350 153.6 MHz (x30), RP2040 128 MHz (x25).
#ifndef PQ_SYS_CLOCK_KHZ
#if PICO_RP2350
#define PQ_SYS_CLOCK_KHZ 153600
#else
#define PQ_SYS_CLOCK_KHZ 128000
#endif
#endif
#ifndef PQ_CORE1_STACK_WORDS
#define PQ_CORE1_STACK_WORDS 2048   // 8 KB
#endif
#ifndef PQ_WATCHDOG_MS
#define PQ_WATCHDOG_MS 1000
#endif

namespace {
RtShared         g_link;
RealtimeTask     g_rt;
Supervisor       g_sup;
Ads131m02Backend g_adc;
PicoSimPacer     g_pacer;
SimAdcBackend    g_sim(&g_pacer);
PicoControlHw    g_ctrl_hw;
PicoAlarmHw      g_alarm_hw;
UsbTransport     g_usb;
UartTransport    g_uart;
FlashConfigStore g_store;
SystemConfig     g_boot_cfg;

uint32_t g_core1_stack[PQ_CORE1_STACK_WORDS];
volatile uint32_t g_core1_alive = 0;
volatile bool g_core1_ready = false;

void core1Main() {
    multicore_lockout_victim_init();   // lets core0 park us during flash writes
    g_rt.init(&g_link, &g_adc, &g_sim, &g_ctrl_hw, g_boot_cfg);
    g_core1_ready = true;
    while (true) {
        g_rt.poll();
        g_core1_alive = g_core1_alive + 1;
    }
}
}  // namespace

int main() {
    set_sys_clock_khz(PQ_SYS_CLOCK_KHZ, true);
    plat::paintStacks(g_core1_stack, PQ_CORE1_STACK_WORDS);

    uint32_t boot = 0;
    if (watchdog_caused_reboot()) boot |= ERR_WATCHDOG_RESET;

    g_usb.init();

    // Configuration: flash -> validated, else compiled defaults.
    g_boot_cfg = configDefaults();
    StoredConfig sc;
    if (g_store.load(sc)) {
        SystemConfig c;
        if (configUnpack(sc, c)) g_boot_cfg = c;
        else boot |= ERR_CONFIG_INVALID;   // corrupted/old image: run on defaults
    }
    if (g_boot_cfg.comm.uart_enable) g_uart.init(g_boot_cfg.comm.uart_baud);

    if (!Classifier_Init()) boot |= ERR_ML_INIT;   // DSP-only operation continues

    multicore_launch_core1_with_stack(core1Main, g_core1_stack, sizeof(g_core1_stack));
    const uint64_t t0 = time_us_64();
    while (!g_core1_ready && time_us_64() - t0 < 2000000) tight_loop_contents();

    SupervisorEnv env;
    env.link = &g_link;
    env.usb = &g_usb;
    env.uart = g_boot_cfg.comm.uart_enable ? &g_uart : nullptr;
    env.store = &g_store;
    env.alarm_hw = &g_alarm_hw;
    env.boot_errors = boot;
    g_sup.init(env, g_boot_cfg);

    watchdog_enable(PQ_WATCHDOG_MS, true);   // pauses while debugging
    uint32_t last_alive = 0;
    uint64_t last_alive_change = time_us_64();
    while (true) {
        g_sup.poll();
        // Feed the watchdog only while BOTH cores make progress.
        const uint32_t a = g_core1_alive;
        const uint64_t now = time_us_64();
        if (a != last_alive) {
            last_alive = a;
            last_alive_change = now;
        }
        if (now - last_alive_change < 200000) watchdog_update();
    }
}
