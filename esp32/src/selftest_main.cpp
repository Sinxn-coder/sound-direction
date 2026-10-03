// ESP32-S3 self-test for the sound-direction core.
//
// No microphones yet: builds known two-channel test buffers in RAM (broadband
// noise with a known inter-mic delay / level difference), runs them through the
// same DirectionEngine used on the desktop, and prints results over Serial.
//
// Sign convention (same as desktop): tdoa = t(left) - t(right).
//   tdoa < 0 -> left mic first -> LEFT,  tdoa > 0 -> right mic first -> RIGHT.

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "direction/DirectionEngine.h"
#include "direction/TemporalSmoother.h"

using namespace sdl;

namespace {

constexpr int kPad = 64; // headroom so delayed reads stay inside the source buffer

struct TestCase {
    const char* name;
    int sampleRate;
    int blockSize;
    int tdoaSamples;   // integer delay between mics
    float farEarDb;    // far mic attenuation
    float amplitude;   // 0 -> silence
    float dcLeft, dcRight;
    float noiseAmp;    // independent uniform noise per mic
    Direction expected;
};

const TestCase kCases[] = {
    {"LEFT  -14 smp, far -6 dB",          48000, 2048, -14, 6.0f, 0.3f, 0.0f, 0.0f, 0.00f, Direction::Left},
    {"RIGHT +12 smp, far -4 dB",          48000, 2048, +12, 4.0f, 0.3f, 0.0f, 0.0f, 0.00f, Direction::Right},
    {"CENTER  0 smp",                     48000, 2048,   0, 0.0f, 0.3f, 0.0f, 0.0f, 0.00f, Direction::Center},
    {"RIGHT +27 smp (~70 deg head model)",48000, 2048, +27, 9.4f, 0.3f, 0.0f, 0.0f, 0.00f, Direction::Right},
    {"LEFT  -20 smp, DC offset + noise",  48000, 2048, -20, 3.0f, 0.3f, 0.4f, -0.2f, 0.05f, Direction::Left},
    {"SILENCE",                           48000, 2048,   0, 0.0f, 0.0f, 0.0f, 0.0f, 0.00002f, Direction::Silent},
    {"LEFT  -5 smp @ 16 kHz, block 1024", 16000, 1024,  -5, 3.0f, 0.3f, 0.0f, 0.0f, 0.00f, Direction::Left},
};

// Deterministic PRNG (no <random> needed on the MCU).
struct XorShift32 {
    uint32_t s;
    explicit XorShift32(uint32_t seed) : s(seed ? seed : 1u) {}
    float uniform() { // [-1, 1)
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return static_cast<float>(s) * (2.0f / 4294967296.0f) - 1.0f;
    }
};

// Low-passed noise, peak-normalized to 1. Broadband-ish, like the desktop test signal.
void makeSource(std::vector<float>& src, std::size_t n, uint32_t seed) {
    src.resize(n);
    XorShift32 rng(seed);
    float lp = 0.0f, peak = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        lp = 0.55f * rng.uniform() + 0.45f * lp;
        src[i] = lp;
        peak = std::max(peak, std::fabs(lp));
    }
    for (float& v : src) v /= peak;
}

void makeStereo(const TestCase& c, const std::vector<float>& src, std::vector<float>& left,
                std::vector<float>& right) {
    const std::size_t n = static_cast<std::size_t>(c.blockSize);
    left.resize(n);
    right.resize(n);
    const float farGain = std::pow(10.0f, -std::fabs(c.farEarDb) / 20.0f);
    const int d = std::abs(c.tdoaSamples);
    XorShift32 noiseL(111), noiseR(222);

    for (std::size_t i = 0; i < n; ++i) {
        const float nearS = c.amplitude * src[i + kPad];
        const float farS = c.amplitude * farGain * src[i + kPad - d];
        float l, r;
        if (c.tdoaSamples > 0)      { r = nearS; l = farS; }
        else if (c.tdoaSamples < 0) { l = nearS; r = farS; }
        else                        { l = nearS; r = nearS; }
        left[i] = l + c.dcLeft + c.noiseAmp * noiseL.uniform();
        right[i] = r + c.dcRight + c.noiseAmp * noiseR.uniform();
    }
}

std::vector<float> g_src, g_left, g_right;
DirectionEngine g_engine; // default config: 18 cm spacing, same as desktop

int runSelfTest() {
    int passed = 0;
    const int total = sizeof(kCases) / sizeof(kCases[0]);

    Serial.println();
    Serial.println("=== sdl_core ESP32-S3 self-test ===");
    Serial.printf("mic spacing %.3f m, CPU %u MHz, free heap %u bytes\n",
                  g_engine.config().micSpacingMeters, getCpuFrequencyMhz(), ESP.getFreeHeap());

    for (int t = 0; t < total; ++t) {
        const TestCase& c = kCases[t];
        makeStereo(c, g_src, g_left, g_right);

        const uint32_t t0 = micros();
        const DirectionResult r = g_engine.process(g_left, g_right, c.sampleRate);
        const uint32_t us = micros() - t0;

        const bool ok = r.direction == c.expected;
        passed += ok;
        const double blockMs = 1000.0 * c.blockSize / c.sampleRate;
        Serial.printf("[%s] %-36s -> %-7s delay %+7.2f smp (%+.3f ms) conf %.2f | %5lu us (%.1f%% of %.1f ms block)\n",
                      ok ? "PASS" : "FAIL", c.name, toString(r.direction), r.delaySamples, r.delayMs,
                      r.confidence, static_cast<unsigned long>(us), 100.0 * us / (blockMs * 1000.0), blockMs);
        if (!ok) Serial.printf("       expected %s\n", toString(c.expected));
    }

    // Temporal smoother: LEFT blocks, a short silence, LEFT again -> should hold LEFT.
    {
        TemporalSmoother smoother(g_engine.config());
        const TestCase& leftCase = kCases[0];
        const TestCase& silentCase = kCases[5];
        const bool pattern[] = {true, true, true, true, false, false, true, true};
        bool ok = true;
        Serial.print("[....] smoother LEFT x4, SILENT x2, LEFT x2 -> ");
        for (bool active : pattern) {
            const TestCase& c = active ? leftCase : silentCase;
            makeStereo(c, g_src, g_left, g_right);
            const DirectionResult s = smoother.update(g_engine.process(g_left, g_right, c.sampleRate), c.sampleRate);
            Serial.printf("%s ", toString(s.direction));
            ok = ok && s.direction == Direction::Left;
        }
        Serial.println(ok ? " PASS" : " FAIL");
        passed += ok;
    }

    Serial.printf("=== %d / %d passed ===\n", passed, total + 1);
    return passed == total + 1 ? 0 : 1;
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(1500); // give USB-CDC / the serial monitor time to attach

    // Pre-allocate so the timed process() calls do not include heap growth.
    makeSource(g_src, 2048 + 2 * kPad, 42);
    g_left.reserve(2048);
    g_right.reserve(2048);
    makeStereo(kCases[0], g_src, g_left, g_right); // non-silent, so all scratch gets allocated
    g_engine.process(g_left, g_right, kCases[0].sampleRate);

    runSelfTest();
}

void loop() {
    delay(10000);
    runSelfTest(); // repeat so a late-attached monitor still sees results
}
