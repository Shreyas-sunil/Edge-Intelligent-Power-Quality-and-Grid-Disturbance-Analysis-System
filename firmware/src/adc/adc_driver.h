// C-style façade over the selected AdcBackend, giving the
// ADC_Init / ADC_Start / ADC_ReadSamples / ADC_Stop API from the specification.
#pragma once
#include "adc/adc_interface.h"

namespace pq {

AdcStatus ADC_Init(AdcBackend* backend, const AdcHwConfig& cfg);
AdcStatus ADC_Start();
bool      ADC_ReadSamples(RawBlock& out);   // non-blocking, true when a block was read
void      ADC_Stop();
AdcStatus ADC_Poll();
AdcBackend* ADC_Backend();
bool      ADC_IsRunning();

}  // namespace pq
