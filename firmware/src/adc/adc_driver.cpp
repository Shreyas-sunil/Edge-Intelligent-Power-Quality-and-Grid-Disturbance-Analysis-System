#include "adc/adc_driver.h"

namespace pq {

namespace {
AdcBackend* g_backend = nullptr;
bool        g_running = false;
}  // namespace

const char* adcStatusName(AdcStatus s) {
    switch (s) {
        case AdcStatus::Ok: return "OK";
        case AdcStatus::NotInitialized: return "NOT_INITIALIZED";
        case AdcStatus::CommFailure: return "COMM_FAILURE";
        case AdcStatus::BadId: return "BAD_ID";
        case AdcStatus::ConfigMismatch: return "CONFIG_MISMATCH";
        case AdcStatus::Timeout: return "TIMEOUT";
        case AdcStatus::Overrun: return "OVERRUN";
        case AdcStatus::InvalidParam: return "INVALID_PARAM";
        case AdcStatus::Unsupported: return "UNSUPPORTED";
    }
    return "UNKNOWN";
}

AdcStatus ADC_Init(AdcBackend* backend, const AdcHwConfig& cfg) {
    if (g_backend && g_running) g_backend->stop();
    g_running = false;
    g_backend = backend;
    if (!g_backend) return AdcStatus::InvalidParam;
    return g_backend->init(cfg);
}

AdcStatus ADC_Start() {
    if (!g_backend) return AdcStatus::NotInitialized;
    AdcStatus s = g_backend->start();
    g_running = (s == AdcStatus::Ok);
    return s;
}

bool ADC_ReadSamples(RawBlock& out) {
    return g_backend && g_running && g_backend->readBlock(out);
}

void ADC_Stop() {
    if (g_backend) g_backend->stop();
    g_running = false;
}

AdcStatus ADC_Poll() { return g_backend ? g_backend->poll() : AdcStatus::NotInitialized; }
AdcBackend* ADC_Backend() { return g_backend; }
bool ADC_IsRunning() { return g_running; }

}  // namespace pq
