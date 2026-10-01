// pq_bench — runs the same kernel micro-benchmarks as the BENCH command, plus a
// full-pipeline throughput measurement, on the build machine. Numbers printed
// here describe THIS CPU; run `BENCH` on the Pico for on-target figures.
#include <cstdio>
#include <cstring>

#include "adc/adc_calibration.h"
#include "adc/sim_adc.h"
#include "app/benchmark.h"
#include "app/pq_engine.h"
#include "common/platform.h"
#include "system/system_monitor.h"
#include "tinyml/classifier.h"

using namespace pq;

int main() {
    SystemConfig cfg = configDefaults();
    const bool ml = Classifier_Init();
    std::printf("platform: %s   ML: %s\n", plat::platformName(), ml ? "ready" : Classifier_Info().error);

    BenchReport br;
    runBenchmarks(cfg.acq.sample_rate_hz, cfg.acq.decimation, br);
    std::printf("\nFFT size selection (analysis rate %.0f Hz, hop = N/2, 2 channels)\n", br.analysis_fs);
    std::printf("   N   res[Hz] window[ms]  fft[us]  analysis[us]  load[%%]\n");
    for (const FftBench& f : br.fft)
        std::printf("%5d %8.2f %9.1f %8u %12u %8.3f\n", f.n, f.resolution_hz, f.window_ms, f.fft_us, f.analysis_us,
                    f.budget_pct);
    const KernelBench& k = br.k;
    std::printf("\nStreaming kernels [ns/sample]: cal %.1f  dcblock %.1f  freq %.1f  rms %.1f  haar %.1f  db4 %.1f  "
                "transient(db4) %.1f  decim %.1f\n  total %.1f ns/sample = %.3f %% of the sample period\n",
                k.calibration_ns, k.dc_block_ns, k.frequency_ns, k.rms_ns, k.haar_ns, k.db4_ns, k.transient_ns,
                k.decimator_ns, k.per_sample_total_ns, k.per_sample_budget_pct);

    // Full pipeline throughput on 20 s of simulated signal with disturbances.
    static FreeRunPacer pacer;
    static SimAdcBackend sim(&pacer);
    static Calibrator cal;
    static PqEngine eng;
    static RawBlock rb;
    static CalibratedBlock cb;
    static EngineOutput out;
    AdcHwConfig hw{cfg.acq.sample_rate_hz, {1, 1}, true};
    sim.init(hw);
    sim.start();
    Rng r(3);
    Disturbance d;
    const char* sc[] = {"sag", "harmonic", "transient", "load"};
    for (int j = 0; j < 4; ++j) {
        makeScenario(sc[j], 2.0 + 4.0 * j, 0, 0, r, d);
        sim.generator().addDisturbance(d);
    }
    cal.configure(cfg.cal, cfg.acq.sample_rate_hz, 50.0f);
    eng.configure(cfg, cfg.acq.sample_rate_hz);
    TimingStats ts;
    uint64_t proc = 0;
    const uint32_t nblocks = static_cast<uint32_t>(20.0 * cfg.acq.sample_rate_hz / kBlockSize);
    for (uint32_t b = 0; b < nblocks; ++b) {
        sim.readBlock(rb);
        const uint64_t t0 = plat::timeUs();
        cal.apply(rb, cb);
        eng.processBlock(cb, out);
        proc += plat::timeUs() - t0;
        if (out.has_frame) ts.add(out.frame);
    }
    std::printf("\nPipeline (20 s of signal, %u frames): %.1f ms processing -> %.3f %% CPU of real time\n",
                ts.frame.n, proc * 1e-3, 100.0 * proc / 20e6);
    std::printf("  per-hop avg [us]: streaming %.1f | fft+thd %.1f | window %.1f | detect %.1f | features %.1f | "
                "ml %.1f (invoke %.1f) | fusion %.1f | total %.1f (max %u)\n",
                ts.per_sample.avg(), ts.spectral.avg(), ts.window.avg(), ts.detect.avg(), ts.feature.avg(),
                ts.ml_total.avg(), ts.ml_invoke.avg(), ts.fusion.avg(), ts.e2e.avg(), ts.e2e.max);
    const ClassifierInfo& ci = Classifier_Info();
    std::printf("  model %u B flash, arena %u B reserved / %u B used\n", ci.model_bytes, ci.arena_bytes,
                ci.arena_used);
    return 0;
}
