// =============================================================================
//  Abstract ADC backend (hardware-abstraction layer).
//
//  The rest of the firmware only ever sees RawBlock objects in ADC counts; all
//  device-specific register handling lives inside a backend implementation:
//
//    * Ads131m02Backend  (src/adc/ads131m02.cpp)   — real 24-bit SPI ADC, PIO+DMA
//    * SimAdcBackend     (src/adc/sim_adc.cpp)     — waveform simulator
//
//  To support another ADC, implement this interface and select it in main.cpp.
// =============================================================================
#pragma once
#include "adc/adc_types.h"

namespace pq {

class AdcBackend {
public:
    virtual ~AdcBackend() = default;

    virtual const char* name() const = 0;
    virtual AdcStatus   init(const AdcHwConfig& cfg) = 0;   // configure device + SPI
    virtual AdcStatus   start() = 0;                         // start continuous sampling
    virtual void        stop() = 0;

    // Non-blocking. Returns true and fills `out` if a complete block is ready.
    // Must be called from the processing core only (single consumer).
    virtual bool readBlock(RawBlock& out) = 0;

    // Called periodically by the consumer; detects stuck data-ready etc. and
    // attempts recovery. Returns the current health status.
    virtual AdcStatus poll() = 0;

    virtual float   actualSampleRate() const = 0;  // may differ from requested
    virtual int32_t fullScaleCounts() const = 0;   // positive full-scale code
    virtual bool    isSimulated() const { return false; }
    virtual const AdcDiagnostics& diagnostics() const = 0;
};

}  // namespace pq
