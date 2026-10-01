#include "common/math_util.h"

namespace pq {

uint32_t crc32(const void* data, size_t len, uint32_t seed) {
    const auto* p = static_cast<const uint8_t*>(data);
    uint32_t crc = ~seed;
    for (size_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

// CRC-16-CCITT (poly 0x1021), MSB first — the ADS131M0x default output CRC.
uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t init) {
    uint16_t crc = init;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

bool strEqualNoCase(const char* a, const char* b) {
    if (!a || !b) return false;
    for (;; ++a, ++b) {
        char x = *a, y = *b;
        if (x >= 'A' && x <= 'Z') x = static_cast<char>(x - 'A' + 'a');
        if (y >= 'A' && y <= 'Z') y = static_cast<char>(y - 'A' + 'a');
        if (x != y) return false;
        if (!x) return true;
    }
}

uint32_t fnv1a32(const char* s) {
    uint32_t h = 2166136261u;
    while (*s) {
        h ^= static_cast<uint8_t>(*s++);
        h *= 16777619u;
    }
    return h;
}

}  // namespace pq
