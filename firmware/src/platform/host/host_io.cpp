#include "platform/host/host_io.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include "common/platform.h"

#if defined(_WIN32)
#include <io.h>

#include <mutex>
#include <thread>
#else
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace pq {

// ------------------------------------------------------------------ FdTransport
#if defined(_WIN32)
struct FdTransport::RxThread {
    std::mutex m;
    std::string q;
    std::thread t;
    explicit RxThread(int fd) : t([this, fd] {
        char c;
        while (::_read(fd, &c, 1) == 1) {
            std::lock_guard<std::mutex> g(m);
            q.push_back(c);
        }
    }) {
        t.detach();
    }
};
#else
struct FdTransport::RxThread {};
#endif

FdTransport::FdTransport(int fd_in, int fd_out, const char* name) : fd_in_(fd_in), fd_out_(fd_out), name_(name) {
#if defined(_WIN32)
    if (fd_in_ >= 0) rx_ = new RxThread(fd_in_);
#else
    if (fd_in_ >= 0) fcntl(fd_in_, F_SETFL, fcntl(fd_in_, F_GETFL) | O_NONBLOCK);
#endif
    tx_.reserve(kCap);
}

size_t FdTransport::read(char* buf, size_t max) {
#if defined(_WIN32)
    if (!rx_) return 0;
    std::lock_guard<std::mutex> g(rx_->m);
    const size_t n = rx_->q.size() < max ? rx_->q.size() : max;
    std::memcpy(buf, rx_->q.data(), n);
    rx_->q.erase(0, n);
    return n;
#else
    if (fd_in_ < 0) return 0;
    const ssize_t n = ::read(fd_in_, buf, max);
    return n > 0 ? static_cast<size_t>(n) : 0;
#endif
}

size_t FdTransport::writeAvailable() const { return kCap - tx_.size(); }

size_t FdTransport::write(const char* data, size_t len) {
    const size_t n = len < writeAvailable() ? len : writeAvailable();
    tx_.insert(tx_.end(), data, data + n);
    return n;
}

void FdTransport::flush() {
    if (tx_.empty() || fd_out_ < 0) return;
#if defined(_WIN32)
    const int n = ::_write(fd_out_, tx_.data(), static_cast<unsigned>(tx_.size()));
#else
    const ssize_t n = ::write(fd_out_, tx_.data(), tx_.size());
#endif
    if (n > 0) tx_.erase(tx_.begin(), tx_.begin() + n);
}

// ------------------------------------------------------------------ MemTransport
size_t MemTransport::read(char* buf, size_t max) {
    const size_t n = in_.size() < max ? in_.size() : max;
    std::memcpy(buf, in_.data(), n);
    in_.erase(0, n);
    return n;
}

size_t MemTransport::write(const char* data, size_t len) {
    out_.append(data, len);
    return len;
}

std::vector<std::string> MemTransport::takeLines() {
    std::vector<std::string> lines;
    size_t p = 0, q;
    while ((q = out_.find('\n', p)) != std::string::npos) {
        lines.emplace_back(out_.substr(p, q - p));
        p = q + 1;
    }
    out_.erase(0, p);
    return lines;
}

int openPty(std::string& slave_path) {
#if defined(_WIN32)
    (void)slave_path;
    return -1;
#else
    const int m = posix_openpt(O_RDWR | O_NOCTTY);
    if (m < 0) return -1;
    if (grantpt(m) != 0 || unlockpt(m) != 0) {
        close(m);
        return -1;
    }
    const char* s = ptsname(m);
    if (!s) {
        close(m);
        return -1;
    }
    slave_path = s;
    // raw mode on the slave side so line endings pass through untouched
    const int sfd = open(s, O_RDWR | O_NOCTTY);
    if (sfd >= 0) {
        termios t{};
        tcgetattr(sfd, &t);
        cfmakeraw(&t);
        tcsetattr(sfd, TCSANOW, &t);
        close(sfd);
    }
    return m;
#endif
}

// ------------------------------------------------------------- FileConfigStore
bool FileConfigStore::load(StoredConfig& out) {
    FILE* f = std::fopen(path_.c_str(), "rb");
    if (!f) return false;
    const size_t n = std::fread(&out, 1, sizeof(out), f);
    std::fclose(f);
    return n == sizeof(out);
}

bool FileConfigStore::save(const StoredConfig& in) {
    FILE* f = std::fopen(path_.c_str(), "wb");
    if (!f) return false;
    const size_t n = std::fwrite(&in, 1, sizeof(in), f);
    std::fclose(f);
    return n == sizeof(in);
}

bool FileConfigStore::erase() { return std::remove(path_.c_str()) == 0 || errno == ENOENT; }

// --------------------------------------------------------------- RealtimePacer
void RealtimePacer::start(uint32_t block_period_us) {
    period_ = block_period_us ? block_period_us : 1;
    t0_ = plat::timeUs();
}

uint32_t RealtimePacer::blocksDue() { return static_cast<uint32_t>((plat::timeUs() - t0_) / period_); }

}  // namespace pq
