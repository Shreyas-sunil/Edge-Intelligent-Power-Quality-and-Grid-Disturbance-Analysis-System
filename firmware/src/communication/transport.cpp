#include "communication/transport.h"

#include <cstring>

namespace pq {

bool LineReader::feed(char c, char* line) {
    if (c == '\n' || c == '\r') {
        if (overlong_) {
            overlong_ = false;
            n_ = 0;
            return false;
        }
        if (n_ == 0) return false;
        std::memcpy(line, buf_, n_);
        line[n_] = 0;
        n_ = 0;
        return true;
    }
    if (overlong_) return false;
    if (n_ + 1 >= kMax) {
        overlong_ = true;
        ++overlong_count_;
        n_ = 0;
        return false;
    }
    buf_[n_++] = c;
    return false;
}

}  // namespace pq
