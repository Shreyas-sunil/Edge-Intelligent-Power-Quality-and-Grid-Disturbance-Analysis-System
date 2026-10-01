// TFLM platform hooks, compiled INTO the pq_tflm library (replaces the port's
// system_setup.cpp / micro_time.cpp). It depends on nothing from the firmware:
// the classifier installs the log/time callbacks at Classifier_Init(), so the
// interpreter's DebugLog goes to our log sink (never raw stdout, which would
// corrupt the JSON-lines protocol).
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>

extern "C" {
void (*pq_tflm_log_hook)(const char* msg) = nullptr;
uint32_t (*pq_tflm_time_hook)(void) = nullptr;

void DebugLog(const char* format, va_list args) {
    if (!pq_tflm_log_hook) return;
    char buf[160];
    vsnprintf(buf, sizeof(buf), format, args);
    pq_tflm_log_hook(buf);
}

int DebugVsnprintf(char* buffer, size_t buf_size, const char* format, va_list vlist) {
    return vsnprintf(buffer, buf_size, format, vlist);
}
}

namespace tflite {
void InitializeTarget() {}
uint32_t ticks_per_second() { return 1000000u; }
uint32_t GetCurrentTimeTicks() { return pq_tflm_time_hook ? pq_tflm_time_hook() : 0u; }
}  // namespace tflite
