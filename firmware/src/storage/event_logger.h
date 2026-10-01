// =============================================================================
//  RAM event log (ring buffer, newest overwrite oldest). Every event produces
//  one entry, updated in place as the event progresses START -> CLASSIFIED ->
//  END, so a single record carries both the DSP detection and the fused/ML
//  classification. O(1) insert; lookup by id scans at most PQ_EVENT_LOG_SIZE.
//  Runs on the supervisor core only -> never interferes with acquisition.
// =============================================================================
#pragma once
#include <cstddef>
#include <cstdint>

#include "power_quality/pq_events.h"
#include "pq_build_config.h"

namespace pq {

class EventLogger {
public:
    void clear();
    void setEnabled(bool e) { enabled_ = e; }
    bool enabled() const { return enabled_; }
    // Insert or update. Returns the stored entry.
    const EventRecord* record(const EventRecord& r);
    size_t count() const { return count_; }
    uint32_t total() const { return total_; }
    uint32_t overwritten() const { return overwritten_; }
    // i = 0 -> oldest retained
    const EventRecord* at(size_t i) const;
    const EventRecord* findById(uint32_t id) const;
    // Human-readable multi-line text block (spec format).
    static size_t formatText(const EventRecord& r, char* out, size_t cap);

private:
    EventRecord e_[PQ_EVENT_LOG_SIZE];
    size_t   head_ = 0;    // next write slot
    size_t   count_ = 0;
    uint32_t total_ = 0;
    uint32_t overwritten_ = 0;
    bool     enabled_ = true;
};

}  // namespace pq
