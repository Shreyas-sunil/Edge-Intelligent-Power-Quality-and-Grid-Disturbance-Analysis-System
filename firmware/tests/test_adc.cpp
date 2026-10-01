// ADC conversion, calibration, ADS131M02 protocol, simulated backend.
#include <cstring>

#include "adc/adc_calibration.h"
#include "adc/ads131m02_protocol.h"
#include "adc/sim_adc.h"
#include "common/math_util.h"
#include "test_framework.h"

using namespace pq;

PQ_TEST(adc, crc16_ccitt_known_vector) {
    const char* s = "123456789";
    PQ_CHECK(crc16Ccitt(reinterpret_cast<const uint8_t*>(s), 9, 0xFFFF) == 0x29B1);   // CRC-16/CCITT-FALSE
}

PQ_TEST(adc, raw_to_engineering_units) {
    CalConfig cc{0.5f, 100.0f, 0.01f, -20.0f, 0.0f};
    Calibrator c;
    c.configure(cc, 10000.0f, 50.0f);
    static RawBlock rb;
    static CalibratedBlock cb;
    std::memset(&rb, 0, sizeof(rb));
    rb.n = kBlockSize;
    for (uint32_t k = 0; k < kBlockSize; ++k) {
        rb.ch[kChVoltage][k] = 100 + static_cast<int32_t>(k) * 2;
        rb.ch[kChCurrent][k] = -20 + static_cast<int32_t>(k) * 100;
    }
    c.apply(rb, cb);
    for (uint32_t k = 0; k < kBlockSize; ++k) {
        PQ_CHECK_NEAR(cb.v[k], k * 1.0, 1e-4);           // (100+2k-100)*0.5
        PQ_CHECK_NEAR(cb.i[k], k * 1.0, 1e-4);           // (-20+100k+20)*0.01
    }
    PQ_CHECK_NEAR(c.rawMean(kChVoltage), 100 + (kBlockSize - 1), 1e-6);
}

PQ_TEST(adc, invalid_samples_are_held) {
    CalConfig cc{1.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    Calibrator c;
    c.configure(cc, 10000.0f, 50.0f);
    static RawBlock rb;
    static CalibratedBlock cb;
    std::memset(&rb, 0, sizeof(rb));
    rb.n = kBlockSize;
    for (uint32_t k = 0; k < kBlockSize; ++k) rb.ch[0][k] = static_cast<int32_t>(k);
    rb.ch[0][10] = 999999;
    rb.markBad(10);
    c.apply(rb, cb);
    PQ_CHECK_NEAR(cb.v[10], 9.0, 1e-6);   // previous good sample
    PQ_CHECK_NEAR(cb.v[11], 11.0, 1e-6);
}

PQ_TEST(adc, phase_compensation_delays_current) {
    // A current leading by 2 deg must come out aligned after phase_deg = 2.
    const float fs = 10000.0f, f = 50.0f;
    CalConfig cc{1.0f, 0.0f, 1.0f, 0.0f, 2.0f};
    Calibrator c;
    c.configure(cc, fs, f);
    PQ_CHECK_NEAR(c.phaseDelaySamples(), 2.0 / 360.0 * fs / f, 1e-4);   // 1.111 samples
    static RawBlock rb;
    static CalibratedBlock cb;
    std::memset(&rb, 0, sizeof(rb));
    rb.n = kBlockSize;
    double max_err = 0;
    for (int blk = 0; blk < 10; ++blk) {
        for (uint32_t k = 0; k < kBlockSize; ++k) {
            const double t = (blk * kBlockSize + k) / fs;
            rb.ch[0][k] = static_cast<int32_t>(1e6 * std::sin(2 * M_PI * f * t));
            rb.ch[1][k] = static_cast<int32_t>(1e6 * std::sin(2 * M_PI * f * t + 2.0 * M_PI / 180.0));
        }
        c.apply(rb, cb);
        if (blk > 1)
            for (uint32_t k = 0; k < kBlockSize; ++k) max_err = std::fmax(max_err, std::fabs(cb.v[k] - cb.i[k]));
    }
    PQ_INFO("max |v - i| after alignment: %.1f counts of 1e6 (%.4f %%)", max_err, max_err / 1e4);
    PQ_CHECK(max_err < 1e6 * 0.002);   // < 0.2 % residual (linear interpolation)
}

PQ_TEST(adc, ads131m02_commands_and_registers) {
    using namespace ads131m02;
    PQ_CHECK(cmdRreg(REG_ID) == 0xA000);
    PQ_CHECK(cmdWreg(REG_CLOCK) == (0x6000 | (0x03 << 7)));
    PQ_CHECK(wregAck(REG_GAIN) == (0x4000 | (0x04 << 7)));
    const OsrChoice o = chooseOsr(10000.0f);
    PQ_CHECK(o.osr == 256);
    PQ_CHECK_NEAR(o.odr, 10000.0, 1e-3);
    PQ_CHECK(chooseOsr(20000.0f).osr == 128);
    PQ_CHECK(chooseOsr(5000.0f).osr == 512);
    PQ_CHECK(clockReg(o.code) == 0x0306);
    PQ_CHECK(gainReg(1, 1) == 0x00);
    PQ_CHECK(gainReg(4, 32) == 0x52);
    uint8_t buf[16];
    const uint16_t d = 0x1234;
    const size_t n = encodeCommand(cmdWreg(REG_MODE), buf, sizeof(buf), &d, 1);
    PQ_CHECK(n == kFrameBytes);
    PQ_CHECK(buf[0] == 0x61 && buf[1] == 0x00 && buf[2] == 0x00 && buf[3] == 0x12 && buf[4] == 0x34);
}

PQ_TEST(adc, ads131m02_frame_decode_and_crc) {
    using namespace ads131m02;
    uint8_t fr[kFrameBytes];
    const uint16_t st = 0x0500 | ST_DRDY0 | ST_DRDY1;
    encodeFrame(st, -1234567, 7654321, fr);
    Frame f = decodeFrame(fr);
    PQ_CHECK(f.crc_ok);
    PQ_CHECK(f.status_ok);
    PQ_CHECK(f.ch[0] == -1234567);
    PQ_CHECK(f.ch[1] == 7654321);
    PQ_CHECK(!f.saturated);
    fr[5] ^= 0x01;   // bit error
    f = decodeFrame(fr);
    PQ_CHECK(!f.crc_ok);
    encodeFrame(st, kFullScalePos, 0, fr);
    PQ_CHECK(decodeFrame(fr).saturated);
    encodeFrame(0x0500, 1, 2, fr);   // DRDY bits missing -> stale data
    PQ_CHECK(!decodeFrame(fr).status_ok);
}

PQ_TEST(adc, sim_backend_blocks_and_timestamps) {
    FreeRunPacer p;
    SimAdcBackend s(&p);
    AdcHwConfig hw{10000.0f, {1, 1}, true};
    PQ_CHECK(s.init(hw) == AdcStatus::Ok);
    PQ_CHECK(s.start() == AdcStatus::Ok);
    static RawBlock rb;
    uint64_t expect = 0;
    for (int k = 0; k < 20; ++k) {
        PQ_CHECK(s.readBlock(rb));
        PQ_CHECK(rb.seq == expect);
        PQ_CHECK(rb.t_first_us == expect * 100);   // 100 us per sample
        PQ_CHECK(rb.n == kBlockSize);
        expect += kBlockSize;
    }
    PQ_CHECK(s.diagnostics().blocks == 20);
    AdcHwConfig bad{50.0f, {1, 1}, true};
    PQ_CHECK(s.init(bad) == AdcStatus::InvalidParam);
}

namespace {
class StepPacer : public SimPacer {
public:
    void start(uint32_t) override { due = 0; }
    void stop() override {}
    uint32_t blocksDue() override { return due; }
    uint64_t startTimeUs() override { return 0; }
    uint32_t due = 0;
};
}  // namespace

PQ_TEST(adc, sim_overrun_is_detected_and_flagged) {
    StepPacer p;
    SimAdcBackend s(&p);
    AdcHwConfig hw{10000.0f, {1, 1}, true};
    s.init(hw);
    s.start();
    static RawBlock rb;
    p.due = 1;
    PQ_CHECK(s.readBlock(rb));
    PQ_CHECK(!s.readBlock(rb));   // nothing due
    p.due = 1 + PQ_ACQ_RING_BLOCKS + 5;   // consumer fell far behind
    PQ_CHECK(s.readBlock(rb));
    PQ_CHECK(rb.flags & RB_GAP_BEFORE);
    PQ_CHECK(s.diagnostics().overruns == PQ_ACQ_RING_BLOCKS + 4);
    PQ_CHECK(s.diagnostics().missing_samples == (PQ_ACQ_RING_BLOCKS + 4) * kBlockSize);
    PQ_CHECK(rb.seq == (1 + PQ_ACQ_RING_BLOCKS + 4) * kBlockSize);   // time base stays exact
}

PQ_TEST(adc, sim_stall_times_out_and_recovers) {
    StepPacer p;
    SimAdcBackend s(&p);
    AdcHwConfig hw{10000.0f, {1, 1}, true};
    s.init(hw);
    s.start();
    static RawBlock rb;
    p.due = 1;
    s.readBlock(rb);
    s.injectStall(true);
    for (uint32_t d = 2; d < 8; ++d) {
        p.due = d;
        PQ_CHECK(!s.readBlock(rb));
    }
    PQ_CHECK(s.poll() == AdcStatus::Timeout);
    s.injectStall(false);
    p.due = 9;
    PQ_CHECK(s.readBlock(rb));
    PQ_CHECK(rb.flags & RB_GAP_BEFORE);
    PQ_CHECK(s.poll() == AdcStatus::Ok);
}

PQ_TEST(adc, sim_crc_fault_injection_marks_samples) {
    FreeRunPacer p;
    SimAdcBackend s(&p);
    AdcHwConfig hw{10000.0f, {1, 1}, true};
    s.init(hw);
    s.injectBadSampleRate(0.05f);
    s.start();
    static RawBlock rb;
    uint32_t bad = 0;
    for (int k = 0; k < 100; ++k) {
        s.readBlock(rb);
        for (uint32_t j = 0; j < rb.n; ++j) bad += rb.isBad(j);
    }
    PQ_CHECK(bad > 150 && bad < 350);   // ~5 % of 5000
    PQ_CHECK(s.diagnostics().crc_errors == bad);
}

PQ_TEST(adc, sim_counts_saturate_at_full_scale) {
    FreeRunPacer p;
    SimAdcBackend s(&p);
    GeneratorParams g;
    g.vnom_rms = 500.0f;   // 707 V peak > 600 V full scale
    s.generator().configure(g, 10000.0f);
    AdcHwConfig hw{10000.0f, {1, 1}, true};
    s.init(hw);
    s.start();
    static RawBlock rb;
    uint32_t sat = 0;
    for (int k = 0; k < 10; ++k) {
        s.readBlock(rb);
        sat += rb.sat_count;
        for (uint32_t j = 0; j < rb.n; ++j) PQ_CHECK(rb.ch[0][j] <= 8388607 && rb.ch[0][j] >= -8388608);
    }
    PQ_CHECK(sat > 0);
    PQ_CHECK(rb.flags & RB_SATURATED || sat > 0);
}
