#include "communication/json_writer.h"

#include <cmath>
#include <cstdio>

namespace pq {

void JsonWriter::reset() {
    n_ = 0;
    first_ = true;
    ovf_ = false;
    if (cap_) b_[0] = 0;
}

void JsonWriter::put(char c) {
    if (n_ + 2 < cap_) {
        b_[n_++] = c;
        b_[n_] = 0;
    } else {
        ovf_ = true;
    }
}

void JsonWriter::raw(const char* s) {
    while (*s) put(*s++);
}

void JsonWriter::str(const char* s) {
    put('"');
    for (; *s; ++s) {
        const char c = *s;
        if (c == '"' || c == '\\') {
            put('\\');
            put(c);
        } else if (static_cast<unsigned char>(c) < 0x20) {
            put(' ');
        } else {
            put(c);
        }
    }
    put('"');
}

void JsonWriter::sep() {
    if (!first_) put(',');
    first_ = false;
}

void JsonWriter::key(const char* k) {
    sep();
    if (k) {
        str(k);
        put(':');
    }
}

JsonWriter& JsonWriter::beginObj(const char* k) {
    key(k);
    put('{');
    first_ = true;
    return *this;
}
JsonWriter& JsonWriter::endObj() {
    put('}');
    first_ = false;
    return *this;
}
JsonWriter& JsonWriter::beginArr(const char* k) {
    key(k);
    put('[');
    first_ = true;
    return *this;
}
JsonWriter& JsonWriter::endArr() {
    put(']');
    first_ = false;
    return *this;
}

JsonWriter& JsonWriter::kv(const char* k, const char* v) {
    key(k);
    str(v ? v : "");
    return *this;
}

static void fmtFloat(char* t, size_t n, double v, int d) {
    if (!std::isfinite(v)) snprintf(t, n, "null");
    else snprintf(t, n, "%.*f", d, v);
}

JsonWriter& JsonWriter::kv(const char* k, float v, int d) { return kv(k, static_cast<double>(v), d); }
JsonWriter& JsonWriter::kv(const char* k, double v, int d) {
    key(k);
    char t[40];
    fmtFloat(t, sizeof(t), v, d);
    raw(t);
    return *this;
}
JsonWriter& JsonWriter::kv(const char* k, int32_t v) {
    key(k);
    char t[16];
    snprintf(t, sizeof(t), "%ld", static_cast<long>(v));
    raw(t);
    return *this;
}
JsonWriter& JsonWriter::kv(const char* k, uint32_t v) {
    key(k);
    char t[16];
    snprintf(t, sizeof(t), "%lu", static_cast<unsigned long>(v));
    raw(t);
    return *this;
}
JsonWriter& JsonWriter::kv(const char* k, uint64_t v) {
    key(k);
    char t[24];
    snprintf(t, sizeof(t), "%llu", static_cast<unsigned long long>(v));
    raw(t);
    return *this;
}
JsonWriter& JsonWriter::kvBool(const char* k, bool v) {
    key(k);
    raw(v ? "true" : "false");
    return *this;
}
JsonWriter& JsonWriter::kvHex(const char* k, uint32_t v) {
    char t[16];
    snprintf(t, sizeof(t), "0x%08lX", static_cast<unsigned long>(v));
    return kv(k, t);
}
JsonWriter& JsonWriter::val(float v, int d) {
    sep();
    char t[40];
    fmtFloat(t, sizeof(t), v, d);
    raw(t);
    return *this;
}
JsonWriter& JsonWriter::val(const char* s) {
    sep();
    str(s);
    return *this;
}
JsonWriter& JsonWriter::val(int32_t v) {
    sep();
    char t[16];
    snprintf(t, sizeof(t), "%ld", static_cast<long>(v));
    raw(t);
    return *this;
}

size_t JsonWriter::finish() {
    if (n_ + 1 < cap_) {
        b_[n_++] = '\n';
        b_[n_] = 0;
    } else {
        ovf_ = true;
    }
    return ovf_ ? 0 : n_;
}

}  // namespace pq
