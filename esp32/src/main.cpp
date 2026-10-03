// ESP32-S3 live sound-direction: two INMP441 mics -> DirectionEngine -> Serial.
//
// Wiring and channel mapping: see Inmp441I2sSource.h.
//   INMP441 #1  L/R = GND   -> I2S slot 0 (WS low)  -> LEFT  buffer
//   INMP441 #2  L/R = 3.3V  -> I2S slot 1 (WS high) -> RIGHT buffer
//   Shared bus: BCLK = GPIO 4, WS = GPIO 5, DATA = GPIO 6.
//
// The direction algorithm is the unchanged shared core in ../../src/direction.

#include <Arduino.h>

#include <cmath>

#include "Inmp441I2sSource.h"
#include "direction/DirectionEngine.h"
#include "direction/TemporalSmoother.h"
#include "utils/DspUtils.h"

using namespace sdl;

namespace {

// ---- Configuration ---------------------------------------------------------
constexpr int kSampleRate = 48000;     // same rate the desktop engine was tested at
constexpr std::size_t kBlockFrames = 1024; // 21.3 ms per direction estimate
constexpr int kPrintEveryBlocks = 4;   // ~85 ms between Serial lines (also on direction change)
constexpr double kMicSpacingM = 0.18;  // measure the real ear-to-ear mic distance and set it here

// INMP441 sensitivity is -26 dBFS @ 94 dB SPL, i.e. dBFS ~= SPL - 120. Normal speech
// (~60 dB SPL) is only ~-60 dBFS, so the desktop default gate (-55 dBFS, tuned for
// normalized WAV files) would call it silence. -72 dBFS ~= 48 dB SPL: above a quiet
// room / mic self-noise (~-85 dBFS), below conversational speech. Tune on site.
constexpr float kSilenceThresholdDbfs = -72.0f;

constexpr std::size_t kStartupDiscardFrames = kSampleRate / 4; // INMP441 needs ~85 ms to settle

// ---- State -------------------------------------------------------------------
Inmp441Config makeMicConfig() {
    Inmp441Config c;
    c.bclkPin = 4;
    c.wsPin = 5;
    c.dataPin = 6;
    c.sampleRate = kSampleRate;
    c.swapChannels = false; // flip only if the startup check shows LEFT/RIGHT reversed
    return c;
}

DirectionConfig makeDirectionConfig() {
    DirectionConfig c;
    c.micSpacingMeters = kMicSpacingM;
    c.silenceThresholdDbfs = kSilenceThresholdDbfs;
    return c;
}

Inmp441I2sSource g_mics(makeMicConfig());
DirectionEngine g_engine(makeDirectionConfig());
TemporalSmoother g_smoother(makeDirectionConfig());
StereoBlock g_block;
bool g_ready = false;
uint32_t g_blockCount = 0;
Direction g_lastPrinted = Direction::Unknown;

void printBanner() {
    const Inmp441Config& m = g_mics.config();
    const DirectionConfig& d = g_engine.config();
    Serial.println();
    Serial.println("=== Sound direction (2x INMP441, shared I2S bus) ===");
    Serial.printf("I2S pins : BCLK=GPIO%d  WS=GPIO%d  DATA=GPIO%d  (port %d, RX master)\n", m.bclkPin,
                  m.wsPin, m.dataPin, static_cast<int>(m.port));
    Serial.println("Mics     : #1 L/R=GND -> LEFT (slot 0), #2 L/R=3.3V -> RIGHT (slot 1)");
    Serial.printf("Audio    : %d Hz, 32-bit slots (24-bit data), block %u frames (%.1f ms)\n",
                  m.sampleRate, static_cast<unsigned>(kBlockFrames), 1000.0 * kBlockFrames / m.sampleRate);
    Serial.printf("Engine   : spacing %.3f m, search +-%d samples, silence gate %.0f dBFS\n",
                  d.micSpacingMeters, d.maxLagSamples(m.sampleRate), d.silenceThresholdDbfs);
    Serial.printf("Channels : %s\n", m.swapChannels ? "SWAPPED (slot1=LEFT)" : "normal (slot0=LEFT)");
}

// Reports per-channel level so wiring problems are obvious at boot:
//   one channel near -inf  -> that mic is missing / SD not connected / L/R pin floating
//   both channels identical -> both mics have the same L/R setting (or SD shorted)
void channelCheck() {
    if (!g_mics.read(g_block, kBlockFrames * 8)) return;
    const double dbL = dsp::toDbfs(dsp::rms(g_block.left));
    const double dbR = dsp::toDbfs(dsp::rms(g_block.right));
    std::size_t identical = 0;
    for (std::size_t i = 0; i < g_block.frames(); ++i) identical += g_block.left[i] == g_block.right[i];
    const double identicalPct = 100.0 * identical / g_block.frames();

    Serial.printf("Mic check: LEFT %.1f dBFS, RIGHT %.1f dBFS, identical samples %.0f%%\n", dbL, dbR,
                  identicalPct);
    if (dbL < -100.0) Serial.println("  WARNING: LEFT channel silent - check mic #1 (L/R must be GND) and SD wiring");
    if (dbR < -100.0) Serial.println("  WARNING: RIGHT channel silent - check mic #2 (L/R must be 3.3V) and SD wiring");
    if (identicalPct > 90.0) Serial.println("  WARNING: channels identical - both mics may share the same L/R setting");
    Serial.println("Tip: tap/scratch near the LEFT mic -> expect LEFT. If you get RIGHT, set swapChannels = true.");
    Serial.println();
}

void printResult(const DirectionResult& s, const DirectionResult& raw, uint32_t processUs) {
    Serial.printf("Direction: %-7s  Confidence: %.2f  Delay: %+6.2f samples (%+.3f ms)  | raw %-7s %5.1f dBFS  %lu us\n",
                  toString(s.direction), s.confidence, s.delaySamples, s.delayMs, toString(raw.direction),
                  raw.levelDbfs, static_cast<unsigned long>(processUs));
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(1500); // let the serial monitor attach
    printBanner();

    const esp_err_t err = g_mics.begin();
    if (err != ESP_OK) {
        Serial.printf("ERROR: I2S init failed: %s (%d)\n", esp_err_to_name(err), static_cast<int>(err));
        return;
    }
    g_block.left.reserve(kBlockFrames * 8);
    g_block.right.reserve(kBlockFrames * 8);

    g_mics.discard(kStartupDiscardFrames);
    channelCheck();
    g_ready = true;
}

void loop() {
    if (!g_ready) {
        delay(1000);
        return;
    }
    if (!g_mics.read(g_block, kBlockFrames)) {
        Serial.println("ERROR: i2s_read failed");
        delay(100);
        return;
    }

    const uint32_t t0 = micros();
    const DirectionResult raw = g_engine.process(g_block.left, g_block.right, g_block.sampleRate);
    const DirectionResult smoothed = g_smoother.update(raw, g_block.sampleRate);
    const uint32_t processUs = micros() - t0;

    ++g_blockCount;
    const bool changed = smoothed.direction != g_lastPrinted;
    if (changed || g_blockCount % kPrintEveryBlocks == 0) {
        printResult(smoothed, raw, processUs);
        g_lastPrinted = smoothed.direction;
    }
}
