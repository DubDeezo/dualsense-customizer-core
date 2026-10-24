// haptic_engine.hpp : header-only, C++17, no external dependencies.
//
// Signal flow (per frame, audio thread):
//   SPSC ring -> sanitize -> drive -> HPF(DC block) -> LPF (4th-order Butterworth)
//             -> stereo-linked compressor -> brickwall limiter -> safety clamp
//             -> interleaved output frame, actuators on channels 3 and 4 (1-based)
//
// Threading contract:
//   * pushFrames()  : exactly ONE producer thread (decoder / capture thread).
//   * render()      : exactly ONE consumer thread (the USB/audio device callback).
//   * setDrive(), stats getters: any thread.
//   render() never allocates, locks, throws, or logs.
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace haptic {

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Lock-free single-producer / single-consumer ring buffer.
// Capacity is rounded up to a power of two. Indices grow monotonically and are
// masked on access, so full/empty are unambiguous and all slots are usable.
// ---------------------------------------------------------------------------
template <typename T>
class SpscRingBuffer {
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");

public:
    explicit SpscRingBuffer(size_t minCapacity)
        : cap_(nextPow2(minCapacity < 2 ? 2 : minCapacity)), mask_(cap_ - 1), buf_(cap_) {}

    size_t capacity() const noexcept { return cap_; }

    // Producer thread only. Returns number of elements actually written.
    size_t write(const T* src, size_t n) noexcept {
        const size_t h = head_.load(std::memory_order_relaxed);
        const size_t t = tail_.load(std::memory_order_acquire);
        const size_t space = cap_ - (h - t);
        n = std::min(n, space);
        if (n == 0) return 0;
        const size_t idx = h & mask_;
        const size_t first = std::min(n, cap_ - idx);
        std::copy(src, src + first, buf_.data() + idx);
        std::copy(src + first, src + n, buf_.data());
        head_.store(h + n, std::memory_order_release);
        return n;
    }

    // Consumer thread only. Returns number of elements actually read.
    size_t read(T* dst, size_t n) noexcept {
        const size_t t = tail_.load(std::memory_order_relaxed);
        const size_t h = head_.load(std::memory_order_acquire);
        n = std::min(n, h - t);
        if (n == 0) return 0;
        const size_t idx = t & mask_;
        const size_t first = std::min(n, cap_ - idx);
        std::copy(buf_.data() + idx, buf_.data() + idx + first, dst);
        std::copy(buf_.data(), buf_.data() + (n - first), dst + first);
        tail_.store(t + n, std::memory_order_release);
        return n;
    }

    // Approximate from either thread (exact from the thread that owns that side).
    size_t size() const noexcept {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }

private:
    static size_t nextPow2(size_t v) noexcept {
        size_t p = 1;
        while (p < v) p <<= 1;
        return p;
    }

    const size_t cap_;
    const size_t mask_;
    std::vector<T> buf_;
    alignas(64) std::atomic<size_t> head_{0};  // written by producer
    alignas(64) std::atomic<size_t> tail_{0};  // written by consumer
};

// ---------------------------------------------------------------------------
// Biquad (RBJ cookbook), Direct Form II Transposed, double precision state.
// Double matters for the low-frequency HPF/LPF at 48 kHz+ sample rates.
// ---------------------------------------------------------------------------
struct BiquadCoeffs {
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;  // a0 normalised to 1

    static BiquadCoeffs lowpass(double fs, double fc, double q) {
        fc = std::min(fc, fs * 0.45);
        const double w0 = 2.0 * kPi * fc / fs;
        const double cw = std::cos(w0), alpha = std::sin(w0) / (2.0 * q);
        const double a0 = 1.0 + alpha;
        BiquadCoeffs c;
        c.b0 = ((1.0 - cw) * 0.5) / a0;
        c.b1 = (1.0 - cw) / a0;
        c.b2 = c.b0;
        c.a1 = (-2.0 * cw) / a0;
        c.a2 = (1.0 - alpha) / a0;
        return c;
    }

    static BiquadCoeffs highpass(double fs, double fc, double q) {
        fc = std::min(fc, fs * 0.45);
        const double w0 = 2.0 * kPi * fc / fs;
        const double cw = std::cos(w0), alpha = std::sin(w0) / (2.0 * q);
        const double a0 = 1.0 + alpha;
        BiquadCoeffs c;
        c.b0 = ((1.0 + cw) * 0.5) / a0;
        c.b1 = -(1.0 + cw) / a0;
        c.b2 = c.b0;
        c.a1 = (-2.0 * cw) / a0;
        c.a2 = (1.0 - alpha) / a0;
        return c;
    }
};

class Biquad {
public:
    void setCoeffs(const BiquadCoeffs& c) noexcept { c_ = c; }
    void reset() noexcept { z1_ = z2_ = 0.0; }

    inline double process(double x) noexcept {
        const double y = c_.b0 * x + z1_;
        z1_ = c_.b1 * x - c_.a1 * y + z2_;
        z2_ = c_.b2 * x - c_.a2 * y;
        // Flush denormals so a silent tail never burns CPU in the callback.
        if (std::fabs(z1_) < 1e-30) z1_ = 0.0;
        if (std::fabs(z2_) < 1e-30) z2_ = 0.0;
        return y;
    }

private:
    BiquadCoeffs c_{};
    double z1_ = 0.0, z2_ = 0.0;
};

// ---------------------------------------------------------------------------
// Stereo-linked compressor (soft knee, dB-domain smoothing) followed by a
// brickwall limiter (instant attack, smoothed release) and a final clamp.
// Linking the detector keeps the left/right actuator balance intact.
// ---------------------------------------------------------------------------
struct DynamicsParams {
    double thresholdDb = -18.0;
    double ratio = 4.0;
    double kneeDb = 6.0;
    double attackMs = 5.0;     // ~2 cycles at 400 Hz; lets the transient punch through
    double releaseMs = 80.0;
    double makeupDb = 6.0;
    double ceiling = 0.80;     // hard output ceiling (linear, full scale = 1.0): hardware safety
    double limiterReleaseMs = 40.0;
};

class StereoDynamics {
public:
    void configure(double fs, const DynamicsParams& p) {
        p_ = p;
        atkCoef_ = std::exp(-1.0 / (0.001 * p.attackMs * fs));
        relCoef_ = std::exp(-1.0 / (0.001 * p.releaseMs * fs));
        limRelCoef_ = std::exp(-1.0 / (0.001 * p.limiterReleaseMs * fs));
        invRatioMinus1_ = 1.0 / p.ratio - 1.0;
        reset();
    }

    void reset() noexcept {
        grDb_ = 0.0;
        limGain_ = 1.0;
    }

    inline void process(float& l, float& r) noexcept {
        // ---- Compressor ----
        const double peak = std::max(std::fabs(double(l)), std::fabs(double(r)));
        const double levelDb = 8.685889638065035 * std::log(std::max(peak, 1e-6));  // 20*log10
        const double over = levelDb - p_.thresholdDb;

        double targetGrDb;  // <= 0
        if (2.0 * over < -p_.kneeDb) {
            targetGrDb = 0.0;
        } else if (2.0 * std::fabs(over) <= p_.kneeDb) {
            const double t = over + p_.kneeDb * 0.5;
            targetGrDb = invRatioMinus1_ * t * t / (2.0 * p_.kneeDb);
        } else {
            targetGrDb = invRatioMinus1_ * over;
        }

        const double coef = (targetGrDb < grDb_) ? atkCoef_ : relCoef_;  // more reduction => attack
        grDb_ = targetGrDb + coef * (grDb_ - targetGrDb);

        const double g = std::exp((grDb_ + p_.makeupDb) * 0.11512925464970229);  // 10^(dB/20)
        double ol = double(l) * g, orr = double(r) * g;

        // ---- Brickwall limiter (no lookahead: gain reacts on the offending sample) ----
        const double pk = std::max(std::fabs(ol), std::fabs(orr));
        const double target = (pk > p_.ceiling) ? (p_.ceiling / pk) : 1.0;
        limGain_ = (target < limGain_) ? target : target + limRelCoef_ * (limGain_ - target);
        ol *= limGain_;
        orr *= limGain_;

        // ---- Final safety clamp (guards float rounding; guarantees |out| <= ceiling) ----
        const double c = p_.ceiling;
        l = float(std::min(std::max(ol, -c), c));
        r = float(std::min(std::max(orr, -c), c));
    }

private:
    DynamicsParams p_{};
    double atkCoef_ = 0, relCoef_ = 0, limRelCoef_ = 0, invRatioMinus1_ = 0;
    double grDb_ = 0.0;
    double limGain_ = 1.0;
};

// ---------------------------------------------------------------------------
// HapticEngine
// ---------------------------------------------------------------------------
struct HapticEngineConfig {
    double sampleRate = 48000.0;
    uint32_t outputChannels = 4;   // channels in the device's interleaved stream
    uint32_t leftActuator = 2;     // 0-based index => "channel 3"
    uint32_t rightActuator = 3;    // 0-based index => "channel 4"
    double lowpassHz = 400.0;
    double highpassHz = 20.0;      // DC / sub-sonic block; actuators can't use it
    size_t ringFrames = 8192;      // ~170 ms at 48 kHz; rounded up to a power of two
    DynamicsParams dynamics{};
};

class HapticEngine {
public:
    struct Frame {
        float l, r;
    };

    explicit HapticEngine(const HapticEngineConfig& cfg)
        : cfg_(cfg), ring_(cfg.ringFrames) {
        if (cfg.sampleRate < 8000.0) throw std::invalid_argument("sampleRate too low");
        if (cfg.leftActuator >= cfg.outputChannels || cfg.rightActuator >= cfg.outputChannels ||
            cfg.leftActuator == cfg.rightActuator)
            throw std::invalid_argument("invalid actuator channel mapping");

        // 4th-order Butterworth low-pass = two cascaded biquads with these Qs.
        const BiquadCoeffs s1 = BiquadCoeffs::lowpass(cfg.sampleRate, cfg.lowpassHz, 0.5411961);
        const BiquadCoeffs s2 = BiquadCoeffs::lowpass(cfg.sampleRate, cfg.lowpassHz, 1.3065630);
        const BiquadCoeffs hp = BiquadCoeffs::highpass(cfg.sampleRate, cfg.highpassHz, 0.7071068);
        for (int ch = 0; ch < 2; ++ch) {
            lp_[ch][0].setCoeffs(s1);
            lp_[ch][1].setCoeffs(s2);
            hp_[ch].setCoeffs(hp);
        }
        dyn_.configure(cfg.sampleRate, cfg.dynamics);
        driveSmoothCoef_ = 1.0f - float(std::exp(-1.0 / (0.010 * cfg.sampleRate)));  // 10 ms
    }

    // Producer thread. `interleavedStereo` holds frames * 2 floats (L,R,L,R...).
    // Returns frames accepted; a shortfall means the ring was full (overrun counted).
    size_t pushFrames(const float* interleavedStereo, size_t frames) noexcept {
        Frame tmp[kBlock];
        size_t pushed = 0;
        while (pushed < frames) {
            const size_t n = std::min(frames - pushed, kBlock);
            for (size_t i = 0; i < n; ++i) {
                tmp[i].l = interleavedStereo[(pushed + i) * 2];
                tmp[i].r = interleavedStereo[(pushed + i) * 2 + 1];
            }
            const size_t w = ring_.write(tmp, n);
            pushed += w;
            if (w < n) {
                overruns_.fetch_add(1, std::memory_order_relaxed);
                break;
            }
        }
        return pushed;
    }

    // Any thread. Pre-compressor gain (0..4). The output ceiling holds regardless.
    void setDrive(float linear) noexcept {
        drive_.store(std::min(std::max(linear, 0.0f), 4.0f), std::memory_order_relaxed);
    }

    // Consumer thread: the audio device callback. Fills `out` with `frames`
    // interleaved frames of cfg.outputChannels. Non-actuator channels are silent.
    void render(float* out, uint32_t frames) noexcept {
        const uint32_t nch = cfg_.outputChannels;
        std::fill_n(out, size_t(frames) * nch, 0.0f);

        const float driveTarget = drive_.load(std::memory_order_relaxed);
        Frame blk[kBlock];

        uint32_t done = 0;
        while (done < frames) {
            const uint32_t n = std::min<uint32_t>(frames - done, uint32_t(kBlock));
            const size_t got = ring_.read(blk, n);
            if (got < n) {
                // Underrun: feed silence through the chain so filter/limiter state
                // decays smoothly instead of the actuators getting a hard step.
                std::fill(blk + got, blk + n, Frame{0.0f, 0.0f});
                underruns_.fetch_add(1, std::memory_order_relaxed);
            }

            for (uint32_t i = 0; i < n; ++i) {
                drive_s_ += driveSmoothCoef_ * (driveTarget - drive_s_);

                float in[2] = {blk[i].l, blk[i].r};
                for (int ch = 0; ch < 2; ++ch) {
                    if (!std::isfinite(in[ch])) in[ch] = 0.0f;  // NaN/Inf guard
                    double x = double(in[ch]) * drive_s_;
                    x = hp_[ch].process(x);
                    x = lp_[ch][0].process(x);
                    x = lp_[ch][1].process(x);
                    in[ch] = float(x);
                }

                dyn_.process(in[0], in[1]);

                float* o = out + size_t(done + i) * nch;
                o[cfg_.leftActuator] = in[0];
                o[cfg_.rightActuator] = in[1];
            }
            done += n;
        }
    }

    // Convenience for C-style device APIs (miniaudio, ALSA, WASAPI wrappers...).
    static void audioCallback(void* userData, float* out, uint32_t frames) noexcept {
        static_cast<HapticEngine*>(userData)->render(out, frames);
    }

    uint64_t underruns() const noexcept { return underruns_.load(std::memory_order_relaxed); }
    uint64_t overruns() const noexcept { return overruns_.load(std::memory_order_relaxed); }
    size_t bufferedFrames() const noexcept { return ring_.size(); }

private:
    static constexpr size_t kBlock = 256;  // stack scratch; 2 KB per block buffer

    HapticEngineConfig cfg_;
    SpscRingBuffer<Frame> ring_;

    Biquad lp_[2][2];
    Biquad hp_[2];
    StereoDynamics dyn_;

    std::atomic<float> drive_{1.0f};
    float drive_s_ = 1.0f;  // smoothed drive, audio thread only
    float driveSmoothCoef_ = 0.0f;

    std::atomic<uint64_t> underruns_{0};
    std::atomic<uint64_t> overruns_{0};
};

}  // namespace haptic
