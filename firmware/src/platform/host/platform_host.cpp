// Host (Linux/macOS/Windows) implementation of the platform layer, used by the
// unit tests, the host simulator and the dataset generator.
#include <chrono>
#include <cstdlib>
#include <thread>

#include "common/platform.h"

namespace pq::plat {

namespace {
const auto g_start = std::chrono::steady_clock::now();
}

uint64_t timeUs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - g_start)
            .count());
}

void sleepMs(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
int coreNum() { return 0; }
MemoryInfo memoryInfo() { return MemoryInfo{}; }
void requestReboot() { std::exit(0); }
const char* platformName() { return "host"; }
uint32_t cpuHz() { return 0; }

}  // namespace pq::plat
