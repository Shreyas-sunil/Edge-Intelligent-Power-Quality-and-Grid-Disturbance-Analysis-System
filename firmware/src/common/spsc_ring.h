// Lock-free single-producer / single-consumer ring used between the two cores
// (core1 = real-time DSP producer, core0 = supervisor consumer, and the reverse
// for commands). Only aligned 32-bit loads/stores with acquire/release ordering
// are used, which is lock-free on Cortex-M0+ (RP2040) and Cortex-M33 (RP2350).
#pragma once
#include <atomic>
#include <cstdint>

namespace pq {

template <typename T, uint32_t N>
class SpscRing {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "SpscRing size must be a power of two");

public:
    bool push(const T& v) {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        const uint32_t t = tail_.load(std::memory_order_acquire);
        if (h - t >= N) {
            ++dropped_;
            return false;
        }
        buf_[h & (N - 1)] = v;
        head_.store(h + 1, std::memory_order_release);
        const uint32_t used = h + 1 - t;
        if (used > high_water_) high_water_ = used;
        return true;
    }

    bool pop(T& v) {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        const uint32_t h = head_.load(std::memory_order_acquire);
        if (h == t) return false;
        v = buf_[t & (N - 1)];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

    uint32_t size() const {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }
    bool     empty() const { return size() == 0; }
    uint32_t capacity() const { return N; }
    uint32_t dropped() const { return dropped_; }       // written by producer only
    uint32_t highWater() const { return high_water_; }  // written by producer only

    void reset() {  // only when both sides are quiescent
        head_.store(0);
        tail_.store(0);
        dropped_ = 0;
        high_water_ = 0;
    }

private:
    T buf_[N];
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    volatile uint32_t dropped_ = 0;
    volatile uint32_t high_water_ = 0;
};

}  // namespace pq
