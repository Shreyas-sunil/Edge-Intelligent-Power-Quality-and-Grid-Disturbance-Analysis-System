// =============================================================================
//  Serial protocol (USB CDC / UART), documented in docs/communication_protocol.md
//
//  Host -> device : one ASCII command per line, case-insensitive keywords.
//  Device -> host : one JSON object per line ("JSON lines"), always carrying
//                   a "type" field: ack, error, status, metrics, event, log,
//                   config, harmonics, features, model, wave, bench, cal, help.
//                   Metrics can alternatively be compact CSV ("M,..." lines).
// =============================================================================
#pragma once
#include <cstddef>
#include <cstdint>

#include "app/pq_engine.h"
#include "communication/json_writer.h"
#include "power_quality/pq_events.h"

namespace pq {

constexpr const char* kFirmwareVersion = "1.0.0";
constexpr int kProtocolVersion = 1;

// Splits `line` in place on whitespace. Returns token count.
int tokenize(char* line, char** tok, int max);
bool tokEq(const char* a, const char* b);   // case-insensitive
bool parseFloat(const char* s, float& out);

const char* errorFlagName(uint32_t bit);
void        writeErrorList(JsonWriter& w, const char* key, uint32_t mask);
const char* eventStateName(uint8_t s);
const char* eventSourceName(uint8_t s);

size_t fmtMetricsJson(char* buf, size_t cap, const FrameResult& f, PqClass event, float confidence,
                      uint64_t ts_ms, uint32_t errors);
size_t fmtMetricsCsv(char* buf, size_t cap, const FrameResult& f, PqClass event, float confidence,
                     uint64_t ts_ms, uint32_t errors);
size_t fmtMetricsCsvHeader(char* buf, size_t cap);
size_t fmtEventJson(char* buf, size_t cap, const EventRecord& r, const char* type = "event");
size_t fmtHarmonicsJson(char* buf, size_t cap, const FrameResult& f);
size_t fmtFeaturesJson(char* buf, size_t cap, const FrameResult& f);

}  // namespace pq
