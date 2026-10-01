#include "common/pq_types.h"

#include <cstring>

#include "common/math_util.h"

namespace pq {

namespace {
const char* const kLong[kNumPqClasses] = {
    "NORMAL", "VOLTAGE_SAG", "VOLTAGE_SWELL", "INTERRUPTION", "HARMONIC_DISTORTION",
    "TRANSIENT", "FREQUENCY_VARIATION", "LOAD_SWITCHING", "COMBINED_EVENT"};
const char* const kShort[kNumPqClasses] = {
    "NORMAL", "SAG", "SWELL", "INTERRUPTION", "HARMONIC",
    "TRANSIENT", "FREQUENCY_VARIATION", "LOAD_SWITCHING", "COMBINED"};
}  // namespace

const char* pqClassName(PqClass c) {
    auto i = static_cast<unsigned>(c);
    return i < static_cast<unsigned>(kNumPqClasses) ? kLong[i] : "UNKNOWN";
}

const char* pqClassShortName(PqClass c) {
    auto i = static_cast<unsigned>(c);
    return i < static_cast<unsigned>(kNumPqClasses) ? kShort[i] : "UNKNOWN";
}

bool pqClassFromString(const char* s, PqClass& out) {
    for (int i = 0; i < kNumPqClasses; ++i) {
        if (strEqualNoCase(s, kLong[i]) || strEqualNoCase(s, kShort[i])) {
            out = static_cast<PqClass>(i);
            return true;
        }
    }
    return false;
}

}  // namespace pq
