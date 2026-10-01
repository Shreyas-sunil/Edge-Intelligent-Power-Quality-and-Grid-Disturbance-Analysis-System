#include "system/log_sink.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "common/platform.h"
#include "common/spsc_ring.h"

namespace pq {

namespace {
SpscRing<LogMsg, 16> g_ring[2];   // one producer per core
}

void logSink(LogLevel lvl, const char* tag, const char* text) {
    LogMsg m;
    m.t_us = plat::timeUs();
    m.level = lvl;
    std::strncpy(m.tag, tag ? tag : "", sizeof(m.tag) - 1);
    m.tag[sizeof(m.tag) - 1] = 0;
    std::strncpy(m.text, text ? text : "", sizeof(m.text) - 1);
    m.text[sizeof(m.text) - 1] = 0;
    g_ring[plat::coreNum() & 1].push(m);
}

void logf(LogLevel lvl, const char* tag, const char* fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    logSink(lvl, tag, buf);
}

bool logPop(LogMsg& out) { return g_ring[0].pop(out) || g_ring[1].pop(out); }

uint32_t logDropped() { return g_ring[0].dropped() + g_ring[1].dropped(); }

const char* logLevelName(LogLevel l) {
    switch (l) {
        case LogLevel::Debug: return "debug";
        case LogLevel::Info: return "info";
        case LogLevel::Warn: return "warn";
        case LogLevel::Error: return "error";
    }
    return "?";
}

}  // namespace pq
