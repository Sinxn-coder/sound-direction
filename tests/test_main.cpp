// Minimal self-contained test runner (no external framework needed at a hackathon).

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "audio/WavFile.h"
#include "direction/DirectionEngine.h"
#include "direction/TemporalSmoother.h"
#include "simulation/BinauralSimulator.h"
#include "simulation/SignalGenerator.h"

using namespace sdl;

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                                   \
    do {                                                                              \
        ++g_checks;                                                                   \
        if (!(cond)) {                                                                \
            ++g_failures;                                                             \
            std::printf("    FAILED: %s  (%s:%d)\n", #cond, __FILE__, __LINE__);      \
        }                                                                             \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                         \
    do {                                                                              \
        ++g_checks;                                                                   \
        const double va_ = (a), vb_ = (b);                                            \
        if (std::fabs(va_ - vb_) > (tol)) {                                           \
            ++g_failures;                                                             \
            std::printf("    FAILED: %s = %.4f, expected %.4f +- %.4f  (%s:%d)\n", #a, \
                        va_, vb_, static_cast<double>(tol), __FILE__, __LINE__);      \
        }                                                                             \
    } while (0)

constexpr int kRate = 48000;

std::vector<float> testSignal(std::size_t n = 4096, uint32_t seed = 7) {
    // Continuous noise burst (no gaps) so every block has signal.
    return generateNoiseBursts(kRate, static_cast<double>(n) / kRate, seed, 10.0, 0.0);
}

DirectionResult runSim(double tdoa, float levelDb = 0.0f, float snr = INFINITY, uint32_t seed = 7) {
    SimulationParams p;
    p.tdoaSamples = tdoa;
    p.farEarAttenuationDb = levelDb;
    p.snrDb = snr;
    const auto s = simulateTwoMics(testSignal(4096, seed), p);
    DirectionEngine engine;
    return engine.process(s.left, s.right, kRate);
}

void testLeftIntegerDelay() {
    const auto r = runSim(-10.0, 3.0f);
    CHECK(r.direction == Direction::Left);
    CHECK_NEAR(r.delaySamples, -10.0, 0.25);
    CHECK_NEAR(r.delayMs, -10.0 * 1000.0 / kRate, 0.01);
    CHECK(r.confidence > 0.8f);
}

void testRightIntegerDelay() {
    const auto r = runSim(+10.0, 3.0f);
    CHECK(r.direction == Direction::Right);
    CHECK_NEAR(r.delaySamples, 10.0, 0.25);
    CHECK(r.confidence > 0.8f);
}

void testFractionalDelay() {
    for (double d : {-17.3, -4.6, 2.5, 7.4, 21.8}) {
        const auto r = runSim(d);
        CHECK_NEAR(r.delaySamples, d, 0.35);
    }
}

void testCenter() {
    const auto r = runSim(0.0);
    CHECK(r.direction == Direction::Center);
    CHECK_NEAR(r.delaySamples, 0.0, 0.1);
    CHECK(r.confidence > 0.8f);
}

void testAmplitudeOnlyIsCenter() {
    // The engine localizes by timing; a pure level difference has zero TDOA.
    const auto r = runSim(0.0, 6.0f);
    CHECK(r.direction == Direction::Center);
}

void testSilence() {
    std::vector<float> zeros(4096, 0.0f);
    DirectionEngine engine;
    const auto r = engine.process(zeros, zeros, kRate);
    CHECK(r.direction == Direction::Silent);
    CHECK(r.confidence == 0.0f);

    // Very quiet noise below the threshold is also silence.
    auto quiet = testSignal();
    for (float& v : quiet) v *= 1e-4f;
    const auto r2 = engine.process(quiet, quiet, kRate);
    CHECK(r2.direction == Direction::Silent);
}

void testDcOffsetIgnored() {
    SimulationParams p;
    p.tdoaSamples = -12.0;
    auto s = simulateTwoMics(testSignal(), p);
    for (float& v : s.left) v += 0.4f;  // big DC offset on one mic
    for (float& v : s.right) v -= 0.2f;
    DirectionEngine engine;
    const auto r = engine.process(s.left, s.right, kRate);
    CHECK(r.direction == Direction::Left);
    CHECK_NEAR(r.delaySamples, -12.0, 0.25);
}

void testNoiseRobustness() {
    int correct = 0;
    for (uint32_t seed = 1; seed <= 10; ++seed) {
        const auto l = runSim(-15.0, 0.0f, 3.0f, seed);
        const auto r = runSim(+15.0, 0.0f, 3.0f, seed);
        correct += (l.direction == Direction::Left) + (r.direction == Direction::Right);
    }
    CHECK(correct >= 19); // SNR 3 dB, 20 trials
}

void testGainInvariance() {
    // NCC should not care about the absolute level (as long as it is above the gate).
    SimulationParams p;
    p.tdoaSamples = 9.0;
    auto s = simulateTwoMics(testSignal(), p);
    DirectionEngine engine;
    const auto r1 = engine.process(s.left, s.right, kRate);
    for (float& v : s.left) v *= 0.05f;
    for (float& v : s.right) v *= 0.05f;
    const auto r2 = engine.process(s.left, s.right, kRate);
    CHECK_NEAR(r1.delaySamples, r2.delaySamples, 1e-3);
    CHECK_NEAR(r1.peakCorrelation, r2.peakCorrelation, 1e-3);
}

void testHeadModelSweep() {
    const DirectionConfig cfg;
    for (int az = -90; az <= 90; az += 15) {
        const auto p = paramsFromAzimuth(az, cfg.micSpacingMeters, cfg.speedOfSoundMps, kRate);
        // The head model must always fit inside the search range.
        CHECK(std::fabs(p.tdoaSamples) < cfg.maxLagSamples(kRate));
        const auto s = simulateTwoMics(testSignal(), p);
        DirectionEngine engine(cfg);
        const auto r = engine.process(s.left, s.right, kRate);
        if (az <= -15) CHECK(r.direction == Direction::Left);
        else if (az >= 15) CHECK(r.direction == Direction::Right);
        else CHECK(r.direction == Direction::Center);
        CHECK(!r.peakAtSearchEdge);
    }
}

void testSearchRange() {
    DirectionConfig cfg;
    cfg.micSpacingMeters = 0.18;
    // 0.18 m * 1.35 / 343 m/s * 48 kHz = 34.006 -> ceil 35, + 1 extra
    CHECK(cfg.maxLagSamples(48000) == 36);
    cfg.micSpacingMeters = 0.10;
    CHECK(cfg.maxLagSamples(16000) == static_cast<int>(std::ceil(0.10 * 1.35 / 343.0 * 16000)) + 1);
}

void testDelayOutsideRangeLowConfidence() {
    // 80 samples is impossible for an 18 cm array at 48 kHz (max ~36).
    const auto r = runSim(-80.0);
    CHECK(r.peakAtSearchEdge || r.confidence < 0.5f);
}

void testInvalidInput() {
    DirectionEngine engine;
    std::vector<float> a(4096, 0.1f), b(4000, 0.1f);
    CHECK(engine.process(a, b, kRate).direction == Direction::Unknown);
    CHECK(engine.process(a, a, 0).direction == Direction::Unknown);
    std::vector<float> tiny(50, 0.1f);
    CHECK(engine.process(tiny, tiny, kRate).direction == Direction::Unknown);
}

void testToneIsAmbiguous() {
    // 3 kHz tone at 48 kHz has a 16-sample period: inside the +-36 lag range
    // there are several equally good peaks. The engine must not be confident.
    SimulationParams p;
    p.tdoaSamples = -6.0;
    const auto s = simulateTwoMics(generateTone(kRate, 4096.0 / kRate, 3000.0), p);
    DirectionEngine engine;
    const auto r = engine.process(s.left, s.right, kRate);
    CHECK(r.confidence < 0.5f);
}

void testSmootherHoldsThroughShortSilence() {
    DirectionConfig cfg;
    TemporalSmoother sm(cfg);
    DirectionResult left;
    left.direction = Direction::Left;
    left.confidence = 0.9f;
    left.delaySamples = -20.0;
    left.azimuthDeg = -60.0;
    for (int i = 0; i < 5; ++i) sm.update(left, kRate);

    DirectionResult silent;
    silent.direction = Direction::Silent;
    const auto held = sm.update(silent, kRate);
    CHECK(held.direction == Direction::Left);

    DirectionResult last = held;
    for (int i = 0; i < 10; ++i) last = sm.update(silent, kRate);
    CHECK(last.direction == Direction::Silent);
}

void testSmootherConverges() {
    DirectionConfig cfg;
    TemporalSmoother sm(cfg);
    DirectionResult r;
    r.direction = Direction::Right;
    r.confidence = 1.0f;
    r.delaySamples = 15.0;
    DirectionResult out;
    for (int i = 0; i < 20; ++i) out = sm.update(r, kRate);
    CHECK_NEAR(out.delaySamples, 15.0, 0.01);
    CHECK(out.direction == Direction::Right);
}

void testWavRoundTrip() {
    const std::string path = "sdl_test_roundtrip.wav";
    std::vector<float> l = testSignal(1000), r = testSignal(1000, 99);
    std::string err;
    CHECK(writeWav(path, {l, r}, kRate, err));
    WavData wav;
    CHECK(readWav(path, wav, err));
    CHECK(wav.channels == 2);
    CHECK(wav.sampleRate == kRate);
    CHECK(wav.frames() == 1000);
    if (wav.frames() == 1000) {
        double maxErr = 0.0;
        for (std::size_t i = 0; i < 1000; ++i) {
            maxErr = std::fmax(maxErr, std::fabs(wav.channelData[0][i] - l[i]));
            maxErr = std::fmax(maxErr, std::fabs(wav.channelData[1][i] - r[i]));
        }
        CHECK(maxErr < 1.0 / 16384.0); // 16-bit quantization
    }
    std::remove(path.c_str());
}

void testFractionalDelayAccuracy() {
    // Compare against the analytically delayed signal for band-limited content
    // (a short sinc kernel cannot be exact right at Nyquist; 15 kHz @ 48 kHz is fine).
    const double freqs[] = {300.0, 1700.0, 5200.0, 11000.0, 15000.0};
    const double delay = 3.37;
    const std::size_t n = 2048;
    auto synth = [&](double shift) {
        std::vector<float> s(n);
        for (std::size_t i = 0; i < n; ++i) {
            double v = 0.0;
            for (double f : freqs) v += 0.1 * std::sin(2.0 * 3.14159265358979 * f * (i - shift) / kRate);
            s[i] = static_cast<float>(v);
        }
        return s;
    };
    const auto y = fractionalDelay(synth(0.0), delay);
    const auto expected = synth(delay);
    double maxErr = 0.0;
    for (std::size_t i = 64; i < n - 64; ++i) maxErr = std::fmax(maxErr, std::fabs(y[i] - expected[i]));
    CHECK_NEAR(maxErr, 0.0, 0.01); // peak amplitude is 0.5
}

} // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"left integer delay", testLeftIntegerDelay},
        {"right integer delay", testRightIntegerDelay},
        {"fractional delays", testFractionalDelay},
        {"center (zero delay)", testCenter},
        {"amplitude-only -> center", testAmplitudeOnlyIsCenter},
        {"silence detection", testSilence},
        {"DC offset removal", testDcOffsetIgnored},
        {"noise robustness (3 dB SNR)", testNoiseRobustness},
        {"gain invariance (NCC)", testGainInvariance},
        {"head-model azimuth sweep", testHeadModelSweep},
        {"search range from geometry", testSearchRange},
        {"out-of-range delay", testDelayOutsideRangeLowConfidence},
        {"invalid input", testInvalidInput},
        {"pure tone is ambiguous", testToneIsAmbiguous},
        {"smoother holds through silence", testSmootherHoldsThroughShortSilence},
        {"smoother converges", testSmootherConverges},
        {"WAV round trip", testWavRoundTrip},
        {"fractional delay accuracy", testFractionalDelayAccuracy},
    };

    for (const auto& [name, fn] : tests) {
        const int before = g_failures;
        fn();
        std::printf("[%s] %s\n", g_failures == before ? " OK " : "FAIL", name);
    }
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
