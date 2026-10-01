// Tiny cross-core log sink. Any module may call logSink(); messages are queued
// (one SPSC ring per core, never blocking) and emitted by the supervisor as
// {"type":"log",...} protocol lines. If the ring is full the message is dropped
// and counted — logging can never stall the real-time path.
#pragma once
#include <cstdint>

namespace pq {

enum class LogLevel : uint8_t { Debug = 0, Info, Warn, Error };

struct LogMsg {
    uint64_t t_us;
    LogLevel level;
    char     tag[7];
    char     text[96];
};

void        logSink(LogLevel lvl, const char* tag, const char* text);
#if defined(__GNUC__)
#define PQ_PRINTF_FMT(a, b) __attribute__((format(printf, a, b)))
#else
#define PQ_PRINTF_FMT(a, b)
#endif
void        logf(LogLevel lvl, const char* tag, const char* fmt, ...) PQ_PRINTF_FMT(3, 4);
bool        logPop(LogMsg& out);        // consumer (core0 supervisor)
uint32_t    logDropped();
const char* logLevelName(LogLevel l);

}  // namespace pq
