#include "app/pq_engine.h"

#include <cmath>
#include <cstring>

#include "common/math_util.h"
#include "common/platform.h"

namespace pq {

namespace {
inline float wrapPi(float a) {
    while (a > kPi) a -= kTwoPi;
    while (a < -kPi) a += kTwoPi;
    return a;
}
}  // namespace

const char* PqEngine::configure(const SystemConfig& cfg, float fs) {
    if (const char* e = configValidate(cfg)) return e;
    if (cfg.acq.hop * cfg.acq.decimation < static_cast<int>(kBlockSize))
        return "acq.hop * acq.decimation must be >= block size";
    cfg_ = cfg;
    fs_ = fs;
    dec_ = cfg.acq.decimation;
    nfft_ = cfg.acq.fft_size;
    hop_ = cfg.acq.hop;
    nhops_ = nfft_ / hop_;
    vpk_ = kSqrt2 * cfg.pq.nominalVoltage;
    us_per_sample_ = 1e6 / fs;

    dcb_v_.configure(cfg.dsp.dc_block_hz, fs);
    dcb_i_.configure(cfg.dsp.dc_block_hz, fs);
    freq_.configure(fs, cfg.pq.nominalFrequency, cfg.dsp.freq_lpf_factor, cfg.dsp.freq_avg_cycles, vpk_);
    rms_.configure(fs, cfg.pq.nominalFrequency);
    dwt_.configure(cfg.dsp.wavelet_type ? WaveletType::Db4 : WaveletType::Haar, cfg.dsp.wavelet_levels);
    configureTransient();
    fusion_.configure(cfg.fusion);
    ml_ready_ = cfg.ml.ml_enable && Classifier_Info().ready;
    decv_.configure(dec_);
    deci_.configure(dec_);
    if (!an_.init(nfft_, fs / dec_)) return "FFT init failed";
    det_.configure(cfg.pq, fs);
    reset();
    return nullptr;
}

void PqEngine::configureTransient() {
    TransientParams tp{};
    tp.fs = fs_;
    tp.f_nominal = cfg_.pq.nominalFrequency;
    tp.vpk_nominal = vpk_;
    tp.dev_pu = cfg_.pq.transientDevPu;
    tp.impulse_pu = cfg_.pq.impulsePu;
    tp.dvdt_factor = cfg_.pq.dvdtFactor;
    tp.wavelet_k = cfg_.pq.waveletK;
    tp.wavelet_floor_pu = cfg_.pq.waveletFloorPu;
    tp.use_wavelet = cfg_.dsp.wavelet_enable != 0;
    tp.enable = cfg_.dsp.transient_enable != 0;
    trans_.configure(tp);
}

void PqEngine::updateSoftConfig(const SystemConfig& cfg) {
    // Parameters that are safe to change on the fly (no buffer re-layout).
    // Restart-only fields (PF_RESTART_ACQ) keep their running values.
    const bool tr_changed = std::memcmp(&cfg.pq, &cfg_.pq, sizeof(PQConfig)) != 0 ||
                            cfg.dsp.transient_enable != cfg_.dsp.transient_enable;
    const AcqConfig acq = cfg_.acq;
    const DspConfig dsp_old = cfg_.dsp;
    const float fnom = cfg_.pq.nominalFrequency;
    cfg_ = cfg;
    cfg_.acq = acq;
    cfg_.pq.nominalFrequency = fnom;
    cfg_.dsp.dc_block_hz = dsp_old.dc_block_hz;
    cfg_.dsp.freq_lpf_factor = dsp_old.freq_lpf_factor;
    cfg_.dsp.freq_avg_cycles = dsp_old.freq_avg_cycles;
    cfg_.dsp.wavelet_enable = dsp_old.wavelet_enable;
    cfg_.dsp.wavelet_type = dsp_old.wavelet_type;
    cfg_.dsp.wavelet_levels = dsp_old.wavelet_levels;
    vpk_ = kSqrt2 * cfg_.pq.nominalVoltage;
    if (tr_changed) configureTransient();
    det_.setConfig(cfg_.pq);
    fusion_.configure(cfg_.fusion);
    ml_ready_ = cfg_.ml.ml_enable && Classifier_Info().ready;
}

void PqEngine::clearHop(HopStats& h) {
    std::memset(&h, 0, sizeof(h));
    h.vmin_half = 1e30f;
    h.vmax_half = 0.0f;
    h.imin = 1e30f;
    h.imax = 0.0f;
    h.ymin = 1e30f;
    h.ymax = 0.0f;
}

void PqEngine::reset() {
    dcb_v_.reset();
    dcb_i_.reset();
    freq_.reset();
    rms_.reset();
    dwt_.reset();
    trans_.reset();
    decv_.reset();
    deci_.reset();
    det_.reset();
    fusion_.reset();
    std::memset(ring_v_, 0, sizeof(ring_v_));
    std::memset(ring_i_, 0, sizeof(ring_i_));
    ring_w_ = 0;
    dec_count_ = 0;
    hop_count_ = 0;
    for (auto& h : hops_) clearHop(h);
    hop_idx_ = 0;
    hops_since_gap_ = 0;
    per_sample_us_acc_ = 0;
    have_hc_ = false;
    have_origin_ = false;
    warmup_ = warmupSamples();
    for (auto& p : pend_) p.used = false;
    for (auto& c : cls_cache_) c.id = 0;
    advisory_open_ = false;
}

void PqEngine::pushEvent(EngineOutput& out, const EventRecord& r) {
    if (out.nevents < static_cast<int>(sizeof(out.events) / sizeof(out.events[0])))
        out.events[out.nevents++] = r;
    ++stats_.events;
}

void PqEngine::processBlock(const CalibratedBlock& b, EngineOutput& out) {
    const uint64_t t0 = plat::timeUs();
    out.has_frame = false;
    out.nevents = 0;
    out.trip = false;
    out.rms_updated = false;
    uint32_t frame_us = 0;

    if (b.flags & RB_GAP_BEFORE) {
        // Lost samples: every stateful stage restarts; frames are flagged invalid
        // until the analysis window is refilled with contiguous data.
        ++stats_.gaps;
        freq_.reset();
        rms_.reset();
        trans_.reset();
        dwt_.reset();
        decv_.reset();
        deci_.reset();
        hops_since_gap_ = 0;
        have_origin_ = false;
        warmup_ = warmupSamples();
    }
    if (!have_origin_) {
        t_origin_us_ = static_cast<double>(b.t_first_us) - static_cast<double>(b.seq) * us_per_sample_;
        have_origin_ = true;
    }
    const uint64_t block_end_us = sampleToUs(static_cast<double>(b.seq + b.n));
    HopStats* h = &hops_[hop_idx_];
    h->bad += b.bad_count;
    h->sat += b.sat_count;
    if (b.flags & RB_GAP_BEFORE) h->gap = true;
    if (b.sat_count) det_.markClipped();

    const float vn = cfg_.pq.nominalVoltage;
    const float inv_vpk = 1.0f / vpk_;
    const float trip_imp = cfg_.ctrl.trip_impulse_pu * vpk_;
    const bool use_wav = cfg_.dsp.wavelet_enable != 0;

    for (uint32_t k = 0; k < b.n; ++k) {
        const int64_t n = static_cast<int64_t>(b.seq + k);
        const float v = b.v[k], i = b.i[k];
        h->sum_v_raw += v;
        h->sum_i_raw += i;
        const float vc = dcb_v_.process(v);
        const float ic = dcb_i_.process(i);

        // --- frequency + cycle RMS
        const CrossingEvent ce = freq_.process(vc, n);
        HalfCycleResult hc;
        const bool warm = warmup_ > 0;
        if (warm) --warmup_;
        if (rms_.process(vc, ic, n, ce, hc) && !warm) {
            const uint64_t now = block_end_us + (plat::timeUs() - t0);
            // Onset estimate = start of the half cycle that first showed the change.
            det_.onHalfCycle(hc, sampleToUs(hc.t_end - hc.len), sampleToUs(hc.t_end), now);
            last_hc_ = hc;
            have_hc_ = true;
            out.rms_updated = true;
            const float u = hc.v_rms / vn;
            if (hc.v_rms < h->vmin_half) h->vmin_half = hc.v_rms;
            if (hc.v_rms > h->vmax_half) h->vmax_half = hc.v_rms;
            if (hc.i_rms < h->imin) h->imin = hc.i_rms;
            if (hc.i_rms > h->imax) h->imax = hc.i_rms;
            ++h->nhalf;
            h->hc_ev += hc.e_v;
            h->hc_ei += hc.e_i;
            h->hc_len += hc.len;
            if (u > 0.5f) {
                const float y = hc.i_rms / u;
                if (y < h->ymin) h->ymin = y;
                if (y > h->ymax) h->ymax = y;
                h->ysum += y;
                ++h->ny;
            }
            if (cfg_.ctrl.armed && !out.trip) {
                if (u > cfg_.ctrl.trip_swell_pu) {
                    out.trip = true;
                    out.trip_class = PqClass::Swell;
                    out.trip_value = u;
                } else if (cfg_.ctrl.trip_on_interruption && u < cfg_.pq.interruptionThreshold) {
                    out.trip = true;
                    out.trip_class = PqClass::Interruption;
                    out.trip_value = u;
                }
            }
        }

        // --- wavelet + transient (sample-level, no window delay)
        const uint32_t produced = use_wav ? dwt_.push(vc) : 0u;
        auto tr = trans_.process(vc, n, freq_.periodSamples(), use_wav ? &dwt_ : nullptr, produced);
        if (warm) tr = TransientDetector::kNone;
        if (tr == TransientDetector::kStarted) {
            ++stats_.transients_raw;
            const uint64_t now = block_end_us + (plat::timeUs() - t0);
            det_.onTransientStart(trans_.info(), sampleToUs(static_cast<double>(n)), now);
        } else if (tr == TransientDetector::kEnded) {
            const TransientInfo& ti = trans_.info();
            det_.onTransientEnd(ti, sampleToUs(static_cast<double>(ti.n_start)),
                                sampleToUs(static_cast<double>(ti.n_end)));
        }
        if (cfg_.ctrl.armed && !out.trip && trans_.active() && std::fabs(vc) > trip_imp) {
            out.trip = true;
            out.trip_class = PqClass::Transient;
            out.trip_value = std::fabs(vc) * inv_vpk;
        }

        // --- window statistics (normalised moments for kurtosis)
        const float x = vc * inv_vpk;
        const float x2 = x * x;
        h->s1 += x;
        h->s2 += x2;
        h->s3 += x2 * x;
        h->s4 += x2 * x2;
        h->ev += vc * vc;
        h->ei += ic * ic;
        h->evi += vc * ic;
        const float av = std::fabs(vc);
        if (av > h->peak) h->peak = av;
        ++h->n;

        // --- anti-alias decimation into the analysis ring
        float yv, yi;
        const bool got = decv_.push(vc, yv);
        deci_.push(ic, yi);
        if (got) {
            ring_v_[ring_w_] = yv;
            ring_i_[ring_w_] = yi;
            ring_w_ = (ring_w_ + 1 == nfft_) ? 0 : ring_w_ + 1;
            ++dec_count_;
            if (++hop_count_ >= hop_) {
                hop_count_ = 0;
                for (int j = 1; j <= dwt_.levels(); ++j) h->wave_e += dwt_.energy(j);
                dwt_.clearEnergy();
                h->dev_e = trans_.devEnergy();
                trans_.clearEnergy();
                ++hops_since_gap_;
                if (dec_count_ >= static_cast<uint32_t>(nfft_)) {
                    const uint64_t tf = plat::timeUs();
                    last_sample_ = n;
                    analyzeFrame(block_end_us, t0, out);
                    frame_us += static_cast<uint32_t>(plat::timeUs() - tf);
                }
                hop_idx_ = (hop_idx_ + 1) % nhops_;
                clearHop(hops_[hop_idx_]);
                h = &hops_[hop_idx_];
            }
        }
    }

    // --- event notes from the detector (+ attach fused classification if known)
    EventRecord notes[DisturbanceDetector::kMaxNotes];
    const int nn = det_.drain(notes, DisturbanceDetector::kMaxNotes);
    for (int k = 0; k < nn; ++k) {
        EventRecord& r = notes[k];
        if (r.state == EV_START) {
            for (auto& p : pend_) {
                if (!p.used) {
                    p.used = true;
                    p.rec = r;
                    break;
                }
            }
        } else if (r.state == EV_END) {
            for (auto& p : pend_)      // still unclassified: keep pending with end info
                if (p.used && p.rec.id == r.id) {
                    p.rec.duration_ms = r.duration_ms;
                    p.rec.magnitude = r.magnitude;
                    p.rec.flags |= r.flags;
                }
            for (const auto& c : cls_cache_) {
                if (c.id == r.id) {
                    // the DSP type may have escalated (sag -> interruption)
                    r.final_type = c.final_type == PqClass::Combined ? PqClass::Combined : r.dsp_type;
                    r.ml_type = c.ml_type;
                    r.ml_conf = c.ml_conf;
                    r.fused_conf = c.fused_conf;
                    r.source = c.source;
                    r.flags |= c.flags;
                    r.cls_latency_ms = c.cls_latency_ms;
                }
            }
        }
        pushEvent(out, r);
    }

    out.vrms_half = have_hc_ ? last_hc_.v_rms : 0.0f;
    out.vpu_half = out.vrms_half / vn;
    out.voltage_state = det_.voltageState();
    if (out.trip) det_.markTripped(out.trip_class);

    const uint32_t total = static_cast<uint32_t>(plat::timeUs() - t0);
    per_sample_us_acc_ += total - frame_us;
    out.block_us = total;
    if (total > stats_.max_block_us) stats_.max_block_us = total;
    stats_.samples += b.n;
    ++stats_.blocks;
}

void PqEngine::analyzeFrame(uint64_t now_base_us, uint64_t proc_start_us, EngineOutput& out) {
    const uint64_t tf0 = plat::timeUs();
    FrameResult& fr = out.frame;
    PqMetrics& m = fr.m;
    std::memset(&m, 0, sizeof(m));
    fr.frame_idx = frame_idx_++;
    fr.t_end_us = sampleToUs(static_cast<double>(last_sample_ + 1));
    fr.window_start_us = fr.t_end_us - static_cast<uint64_t>(nfft_ * dec_ * us_per_sample_);

    // ---------------------------------------------------------- spectral
    const float f0 = freq_.valid() ? freq_.frequency() : -1.0f;
    const float fnom = cfg_.pq.nominalFrequency;
    const int maxh = cfg_.dsp.max_harmonic;
    HarmonicResult hv, hi;
    for (int k = 0, r = ring_w_; k < nfft_; ++k, r = (r + 1 == nfft_) ? 0 : r + 1) lin_[k] = ring_v_[r];
    an_.analyze(lin_, f0, fnom, maxh, hv);
    for (int k = 0, r = ring_w_; k < nfft_; ++k, r = (r + 1 == nfft_) ? 0 : r + 1) lin_[k] = ring_i_[r];
    an_.analyze(lin_, hv.f0, fnom, maxh, hi);   // same f0 -> phases comparable
    const uint64_t tf1 = plat::timeUs();

    m.freq = freq_.valid() ? freq_.frequency() : 0.0f;
    m.freq_valid = freq_.valid();
    m.rocof = freq_.rocof();
    m.fft_freq = hv.fft_peak_hz;
    m.harm_valid = hv.valid;
    m.thd_v = hv.thd_pct;
    m.thd_i = hi.valid ? hi.thd_pct : 0.0f;
    for (int hh = 1; hh <= PQ_REPORT_HARMONICS; ++hh) {
        m.v_h[hh] = hh <= hv.h_max ? hv.mag[hh] : 0.0f;
        m.i_h[hh] = hh <= hi.h_max ? hi.mag[hh] : 0.0f;
        m.i_ph[hh] = wrapPi(hi.phase[hh] - hh * hv.phase[1]);
    }
    const float phi = wrapPi(hv.phase[1] - hi.phase[1]);
    m.phi_vi_deg = phi * (180.0f / kPi);
    m.dpf = (hv.valid && hi.valid) ? std::cos(phi) : 0.0f;
    m.npeaks = static_cast<uint8_t>(hv.npeaks);
    for (int k = 0; k < 3; ++k) m.peaks[k] = hv.peaks[k];

    // ---------------------------------------------------- window statistics
    HopStats w;
    clearHop(w);
    for (int k = 0; k < nhops_; ++k) {
        const HopStats& s = hops_[k];
        w.s1 += s.s1; w.s2 += s.s2; w.s3 += s.s3; w.s4 += s.s4;
        w.ev += s.ev; w.ei += s.ei; w.evi += s.evi;
        w.sum_v_raw += s.sum_v_raw; w.sum_i_raw += s.sum_i_raw;
        w.peak = s.peak > w.peak ? s.peak : w.peak;
        w.wave_e += s.wave_e; w.dev_e += s.dev_e;
        if (s.nhalf) {
            w.vmin_half = s.vmin_half < w.vmin_half ? s.vmin_half : w.vmin_half;
            w.vmax_half = s.vmax_half > w.vmax_half ? s.vmax_half : w.vmax_half;
            w.imin = s.imin < w.imin ? s.imin : w.imin;
            w.imax = s.imax > w.imax ? s.imax : w.imax;
        }
        if (s.ny) {
            w.ymin = s.ymin < w.ymin ? s.ymin : w.ymin;
            w.ymax = s.ymax > w.ymax ? s.ymax : w.ymax;
            w.ysum += s.ysum;
            w.ny += s.ny;
        }
        w.hc_ev += s.hc_ev; w.hc_ei += s.hc_ei; w.hc_len += s.hc_len;
        w.nhalf += s.nhalf; w.n += s.n; w.bad += s.bad; w.sat += s.sat;
        w.gap = w.gap || s.gap;
    }
    const float nn = w.n ? static_cast<float>(w.n) : 1.0f;
    // True RMS over the whole half cycles inside the window (cycle-synchronous:
    // no ripple from a non-integer number of periods). Fallback: raw window.
    if (w.hc_len > 0.5f * nn) {
        m.vrms = std::sqrt(w.hc_ev / w.hc_len);
        m.irms = std::sqrt(w.hc_ei / w.hc_len);
    } else {
        m.vrms = std::sqrt(w.ev / nn);
        m.irms = std::sqrt(w.ei / nn);
    }
    m.p = w.evi / nn;
    m.s = m.vrms * m.irms;
    m.pf = m.s > 1e-6f ? m.p / m.s : 0.0f;
    m.vdc = w.sum_v_raw / nn;
    m.idc = w.sum_i_raw / nn;
    m.vrms_half = have_hc_ ? last_hc_.v_rms : m.vrms;
    m.vmin_half = w.nhalf ? w.vmin_half : m.vrms_half;
    m.vmax_half = w.nhalf ? w.vmax_half : m.vrms_half;
    m.imin_cycle = w.nhalf ? w.imin : m.irms;
    m.imax_cycle = w.nhalf ? w.imax : m.irms;
    m.peak_v = w.peak;
    m.crest = m.vrms > 1e-6f ? w.peak / m.vrms : 0.0f;
    {
        const float m1 = w.s1 / nn, m2 = w.s2 / nn, m3 = w.s3 / nn, m4 = w.s4 / nn;
        const float var = m2 - m1 * m1;
        const float c4 = m4 - 4.0f * m1 * m3 + 6.0f * m1 * m1 * m2 - 3.0f * m1 * m1 * m1 * m1;
        m.kurtosis = var > 1e-9f ? c4 / (var * var) : 0.0f;
    }
    const float ev = w.ev > 1e-9f ? w.ev : 1e-9f;
    m.wavelet_energy = std::log10(w.wave_e / ev + 1e-12f);
    m.transient_energy = std::log10(w.dev_e / ev + 1e-12f);
    fr.y_step = (w.ny >= 2 && w.ysum > 1e-9f) ? (w.ymax - w.ymin) / (w.ysum / w.ny) : 0.0f;

    const bool window_ok = hops_since_gap_ >= static_cast<uint32_t>(nhops_) && !w.gap;
    m.valid = window_ok && w.bad * 100u <= w.n;   // <= 1 % repaired samples tolerated
    uint32_t q = 0;
    if (!window_ok) q |= ERR_MISSING_SAMPLES;
    if (w.bad) q |= ERR_INVALID_SAMPLES;
    if (w.sat) q |= ERR_SATURATION;
    if (!m.freq_valid) q |= ERR_FREQ_LOST;
    m.quality = q;
    if (!m.valid) ++stats_.frames_invalid;
    const uint64_t tf2 = plat::timeUs();

    // ---------------------------------------------------- frame detection
    FrameContext fc{};
    fc.now_us = now_base_us + (plat::timeUs() - proc_start_us);
    fc.window_start_us = fr.window_start_us;
    fc.thd_v = m.thd_v;
    fc.freq = m.freq;
    fc.freq_valid = m.freq_valid && m.valid;
    fc.harm_valid = m.harm_valid && m.valid;
    fc.vmin_pu = m.vmin_half / cfg_.pq.nominalVoltage;
    fc.vmax_pu = m.vmax_half / cfg_.pq.nominalVoltage;
    fc.y_step = fr.y_step;
    fc.vrms = m.vrms;
    fr.evidence = det_.onFrame(fc);
    const uint64_t tf3 = plat::timeUs();

    // ---------------------------------------------------- features + ML
    extractFeatures(m, fr.y_step, cfg_.pq, fr.features);
    const uint64_t tf4 = plat::timeUs();
    if (ml_ready_ && m.valid) fr.ml = Classifier_Run(fr.features.v);
    else fr.ml = ClassificationResult{};
    const uint64_t tf5 = plat::timeUs();

    // ---------------------------------------------------- fusion
    fr.fusion = fusion_.fuse(fr.evidence, fr.ml);
    const uint64_t now = now_base_us + (plat::timeUs() - proc_start_us);

    // ML-only advisory event lifecycle
    if (fr.fusion.flags & EVF_ML_ONLY) {
        if (!advisory_open_) {
            advisory_ = EventRecord{};
            advisory_.id = advisory_id_++;
            advisory_.dsp_type = PqClass::Normal;
            advisory_.final_type = advisory_.ml_type = fr.fusion.final_class;
            advisory_.state = EV_START;
            advisory_.source = SRC_ML;
            advisory_.flags = EVF_ML_ONLY;
            advisory_.t_start_us = fr.window_start_us;
            advisory_.ml_conf = fr.ml.confidence;
            advisory_.fused_conf = fr.fusion.confidence;
            advisory_.vrms = m.vrms;
            advisory_.freq = m.freq;
            advisory_.thd = m.thd_v;
            advisory_.det_latency_ms = advisory_.cls_latency_ms = (now - fr.window_start_us) * 1e-3f;
            advisory_open_ = true;
            pushEvent(out, advisory_);
        }
    } else if (advisory_open_) {
        advisory_.state = EV_END;
        advisory_.duration_ms = static_cast<uint32_t>((fr.t_end_us - advisory_.t_start_us) / 1000u);
        advisory_open_ = false;
        pushEvent(out, advisory_);
    }
    classifyPendingEvents(fr, now, out);
    const uint64_t tf6 = plat::timeUs();

    fr.timing.per_sample_us = per_sample_us_acc_;
    per_sample_us_acc_ = 0;
    fr.timing.spectral_us = static_cast<uint32_t>(tf1 - tf0);
    fr.timing.window_us = static_cast<uint32_t>(tf2 - tf1);
    fr.timing.detect_us = static_cast<uint32_t>(tf3 - tf2);
    fr.timing.feature_us = static_cast<uint32_t>(tf4 - tf3);
    fr.timing.ml_us = static_cast<uint32_t>(tf5 - tf4);
    fr.timing.fusion_us = static_cast<uint32_t>(tf6 - tf5);
    fr.timing.frame_us = static_cast<uint32_t>(tf6 - tf0);
    if (fr.timing.frame_us > stats_.max_frame_us) stats_.max_frame_us = fr.timing.frame_us;
    ++stats_.frames;
    out.has_frame = true;
}

void PqEngine::classifyPendingEvents(const FrameResult& fr, uint64_t now_us, EngineOutput& out) {
    const uint64_t cyc = static_cast<uint64_t>(1e6f / cfg_.pq.nominalFrequency);
    for (auto& p : pend_) {
        if (!p.used) continue;
        EventRecord r = p.rec;
        // Wait until the window has seen at least one cycle after the onset.
        if (fr.t_end_us < r.t_start_us + cyc) continue;
        PqClass d = r.dsp_type;
        // A sag may already have deepened into an interruption by now.
        if (d == PqClass::Sag && (fr.fusion.asserted & classBit(PqClass::Interruption))) {
            d = PqClass::Interruption;
            r.dsp_type = d;
        }
        const int di = static_cast<int>(d);
        r.state = EV_CLASSIFIED;
        r.ml_type = fr.ml.valid ? static_cast<PqClass>(fr.ml.class_id) : PqClass::Normal;
        r.ml_conf = fr.ml.valid ? fr.ml.probs[fr.ml.class_id] : 0.0f;
        r.fused_conf = di < kNumMlClasses ? fr.fusion.fused[di] : fr.fusion.confidence;
        const bool combined = fr.fusion.final_class == PqClass::Combined &&
                              (fr.fusion.asserted & classBit(d));
        r.final_type = combined ? PqClass::Combined : d;
        if (combined) r.flags |= EVF_COMBINED;
        r.components |= fr.fusion.asserted;
        r.source = (fr.ml.valid && fr.ml.class_id == di) ? SRC_DSP_ML : SRC_DSP;
        if (fr.ml.valid && fr.ml.class_id != di && fr.ml.confidence > 0.8f) r.flags |= EVF_ML_DISAGREE;
        r.cls_latency_ms = now_us > r.t_start_us ? (now_us - r.t_start_us) * 1e-3f : 0.0f;
        pushEvent(out, r);
        // remember for the END note
        ClsCache& c = cls_cache_[cls_idx_];
        cls_idx_ = (cls_idx_ + 1) % kClsCache;
        c.id = r.id;
        c.final_type = r.final_type;
        c.ml_type = r.ml_type;
        c.ml_conf = r.ml_conf;
        c.fused_conf = r.fused_conf;
        c.source = r.source;
        c.flags = r.flags;
        c.cls_latency_ms = r.cls_latency_ms;
        p.used = false;
    }
}

}  // namespace pq
