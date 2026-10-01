// Minimal allocation-free JSON line builder (fixed buffer, truncation-safe).
#pragma once
#include <cstddef>
#include <cstdint>

namespace pq {

class JsonWriter {
public:
    JsonWriter(char* buf, size_t cap) : b_(buf), cap_(cap) { reset(); }
    void reset();
    JsonWriter& beginObj(const char* key = nullptr);
    JsonWriter& endObj();
    JsonWriter& beginArr(const char* key = nullptr);
    JsonWriter& endArr();
    JsonWriter& kv(const char* key, const char* v);
    JsonWriter& kv(const char* key, float v, int decimals = 3);
    JsonWriter& kv(const char* key, double v, int decimals);
    JsonWriter& kv(const char* key, int32_t v);
    JsonWriter& kv(const char* key, uint32_t v);
    JsonWriter& kv(const char* key, uint64_t v);
    JsonWriter& kvBool(const char* key, bool v);
    JsonWriter& kvHex(const char* key, uint32_t v);
    JsonWriter& val(float v, int decimals = 3);   // array element
    JsonWriter& val(const char* s);
    JsonWriter& val(int32_t v);
    // Finish with '\n'. Returns length, or 0 if the buffer overflowed.
    size_t finish();
    const char* str() const { return b_; }
    bool overflow() const { return ovf_; }

private:
    void sep();
    void key(const char* k);
    void raw(const char* s);
    void put(char c);
    void str(const char* s);
    char*  b_;
    size_t cap_;
    size_t n_ = 0;
    bool   first_ = true;
    bool   ovf_ = false;
};

}  // namespace pq
