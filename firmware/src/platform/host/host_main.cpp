// =============================================================================
//  pq_sim — the complete firmware application running on a PC.
//
//  Same RealtimeTask + Supervisor + PqEngine + TinyML as the Pico build; the
//  simulated ADC is paced in real time (or as fast as possible with --fast).
//  The serial protocol is spoken on stdin/stdout, or on a pseudo-terminal
//  (--pty) so the Python tools can connect exactly as they would to a Pico:
//
//      ./pq_sim --pty            -> prints "PTY /dev/pts/5"
//      python tools/serial_monitor.py --port /dev/pts/5
//
//  or let the tools spawn it:  python tools/serial_monitor.py --port sim
// =============================================================================
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "app/realtime_task.h"
#include "app/supervisor.h"
#include "common/platform.h"
#include "platform/host/host_io.h"
#include "tinyml/classifier.h"

#include <chrono>
#include <thread>
#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#endif

using namespace pq;

namespace {
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

RtShared g_link;
RealtimeTask g_rt;
Supervisor g_sup;

void usage() {
    std::fprintf(stderr,
                 "usage: pq_sim [--pty] [--fast] [--duration s] [--script \"CMD;CMD\"] [--config file]\n"
                 "  --pty        serve the protocol on a pseudo-terminal (path printed on stderr)\n"
                 "  --fast       do not pace in real time (process as fast as possible)\n"
                 "  --duration   exit after this many simulated seconds\n"
                 "  --script     commands executed at start-up, ';' separated\n"
                 "  --config     config file used by CONFIG SAVE/LOAD (default pq_config.bin)\n");
}
}  // namespace

int main(int argc, char** argv) {
    bool use_pty = false, fast = false;
    double duration = -1.0;
    std::string script, cfg_path = "pq_config.bin";
    for (int k = 1; k < argc; ++k) {
        const std::string a = argv[k];
        if (a == "--pty") use_pty = true;
        else if (a == "--fast") fast = true;
        else if (a == "--duration" && k + 1 < argc) duration = std::atof(argv[++k]);
        else if (a == "--script" && k + 1 < argc) script = argv[++k];
        else if (a == "--config" && k + 1 < argc) cfg_path = argv[++k];
        else {
            usage();
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    int fd_in = 0, fd_out = 1;
#if defined(_WIN32)
    _setmode(0, _O_BINARY);   // no CRLF translation of the protocol stream
    _setmode(1, _O_BINARY);
#endif
    if (use_pty) {
        std::string path;
        const int m = openPty(path);
        if (m < 0) {
            std::fprintf(stderr, "failed to open a pseudo-terminal\n");
            return 1;
        }
        fd_in = fd_out = m;
        std::fprintf(stderr, "PTY %s\n", path.c_str());
        std::fflush(stderr);
    }

    static FdTransport io(fd_in, fd_out, use_pty ? "pty" : "stdio");
    static FileConfigStore store(cfg_path);
    static RecorderControlHw ctrl_hw;
    static NullAlarmHw alarm_hw;
    static RealtimePacer rt_pacer;
    static FreeRunPacer fast_pacer;
    static SimAdcBackend sim(fast ? static_cast<SimPacer*>(&fast_pacer) : &rt_pacer);

    uint32_t boot = 0;
    SystemConfig cfg = configDefaults();
    StoredConfig sc;
    if (store.load(sc)) {
        if (!configUnpack(sc, cfg)) {
            cfg = configDefaults();
            boot |= ERR_CONFIG_INVALID;
        }
    }
    cfg.acq.source = static_cast<uint8_t>(AdcSource::Simulated);   // no hardware on a PC
    if (!Classifier_Init()) boot |= ERR_ML_INIT;

    g_rt.init(&g_link, nullptr, &sim, &ctrl_hw, cfg);
    SupervisorEnv env;
    env.link = &g_link;
    env.usb = &io;
    env.store = &store;
    env.alarm_hw = &alarm_hw;
    env.boot_errors = boot;
    g_sup.init(env, cfg);

    size_t p = 0;
    while (p < script.size()) {
        size_t q = script.find(';', p);
        if (q == std::string::npos) q = script.size();
        const std::string c = script.substr(p, q - p);
        if (!c.empty()) g_sup.command(c.c_str());
        p = q + 1;
    }

    while (!g_stop) {
        const bool worked = g_rt.poll();
        g_sup.poll();
        if (duration > 0 && sim.generator().time() >= duration) break;
        if (!worked && !fast) std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    for (int k = 0; k < 50; ++k) {   // let pending output drain
        g_sup.poll();
        io.flush();
    }
    return 0;
}
