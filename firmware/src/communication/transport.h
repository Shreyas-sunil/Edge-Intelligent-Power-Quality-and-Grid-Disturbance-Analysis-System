// Byte transport abstraction (USB CDC, UART, host stdio/pty). Both calls are
// NON-blocking: the supervisor never waits on a slow host.
#pragma once
#include <cstddef>

namespace pq {

class Transport {
public:
    virtual ~Transport() = default;
    virtual const char* name() const = 0;
    virtual bool connected() const = 0;
    // Returns bytes read (0 if none).
    virtual size_t read(char* buf, size_t max) = 0;
    // Free space for write(); a line is only written if it fits entirely.
    virtual size_t writeAvailable() const = 0;
    virtual size_t write(const char* data, size_t len) = 0;
    virtual void   flush() {}
};

// Accumulates bytes into lines ('\n' or '\r' terminated). Over-long lines are
// discarded (flagged) instead of overflowing.
class LineReader {
public:
    static constexpr size_t kMax = 160;
    // Returns true when `line` holds a complete line.
    bool feed(char c, char* line);
    bool partial() const { return n_ > 0; }
    void clear() { n_ = 0; overlong_ = false; }
    unsigned overlongCount() const { return overlong_count_; }

private:
    char     buf_[kMax];
    size_t   n_ = 0;
    bool     overlong_ = false;
    unsigned overlong_count_ = 0;
};

}  // namespace pq
