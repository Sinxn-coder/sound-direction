#pragma once

// Two INMP441 MEMS microphones on ONE shared I2S bus (ESP32-S3, I2S RX master).
//
// Wiring (both mics share SCK, WS and SD; the L/R pin picks the time slot):
//
//   ESP32-S3            INMP441 #1 (LEFT ear)     INMP441 #2 (RIGHT ear)
//   3.3V  ------------- VDD                        VDD
//   GND   ------------- GND                        GND
//   GPIO 4 (BCLK) ----- SCK                        SCK
//   GPIO 5 (WS)   ----- WS                         WS
//   GPIO 6 (DATA) ----- SD                         SD
//                       L/R = GND  -> LEFT slot    L/R = 3.3V -> RIGHT slot
//
// How the shared data line works: each I2S frame has two 32-bit slots. While WS is
// LOW the mic with L/R=GND drives SD (left slot); while WS is HIGH the mic with
// L/R=3.3V drives SD (right slot). The other mic tri-states its output. Because both
// slots belong to the same frame, LEFT[i] and RIGHT[i] are sampled on the same
// clock edge sequence -> the two channels are inherently sample-synchronized, which
// is exactly what TDOA needs. Never use two separate I2S peripherals for this.
//
// Sample format: each slot is 32 bits; the INMP441 sends 24-bit two's-complement
// audio MSB-first, so the 24 significant bits sit in bits [31:8] of the received
// word and bits [7:0] are zero/undefined. Conversion: (int32_t)word >> 8 gives a
// signed 24-bit value; divide by 2^23 for a float in [-1, 1).

#include <cstddef>
#include <cstdint>
#include <vector>

#include <driver/i2s.h>

#include "audio/AudioSource.h" // header-only IStereoSource interface from the shared core

namespace sdl {

struct Inmp441Config {
    int bclkPin = 4;           // I2S SCK / BCLK
    int wsPin = 5;             // I2S WS / LRCLK
    int dataPin = 6;           // I2S SD (shared by both mics)
    int sampleRate = 48000;    // INMP441 supports ~7.8 - 50 kHz; 48 kHz -> 3.072 MHz BCLK
    i2s_port_t port = I2S_NUM_0;
    int dmaBufCount = 8;       // 8 x 256 frames = 2048 frames (~43 ms) of DMA headroom
    int dmaBufLen = 256;       // frames per DMA buffer
    // Set true only if the startup channel check shows LEFT/RIGHT reversed
    // (e.g. a different core version orders the slots differently).
    bool swapChannels = false;
};

// Converts one received 32-bit I2S slot from an INMP441 to a float in [-1, 1).
inline float inmp441SlotToFloat(int32_t slot) {
    return static_cast<float>(slot >> 8) * (1.0f / 8388608.0f); // arithmetic shift keeps the sign
}

// IStereoSource implementation backed by the ESP32 I2S RX DMA.
// read() blocks until `frames` complete stereo frames are available.
class Inmp441I2sSource : public IStereoSource {
public:
    explicit Inmp441I2sSource(const Inmp441Config& cfg) : cfg_(cfg) {}
    ~Inmp441I2sSource() override { end(); }

    // Installs the I2S driver. Returns an esp_err_t (ESP_OK on success).
    esp_err_t begin();
    void end();

    int sampleRate() const override { return cfg_.sampleRate; }
    bool read(StereoBlock& out, std::size_t frames) override;

    // Reads and throws away `frames` frames (mic power-up settling time).
    bool discard(std::size_t frames);

    const Inmp441Config& config() const { return cfg_; }
    uint64_t framesRead() const { return framesRead_; }

private:
    bool readRaw(std::size_t frames);

    Inmp441Config cfg_;
    bool installed_ = false;
    std::vector<int32_t> raw_; // interleaved slots: [slot0, slot1, slot0, slot1, ...]
    uint64_t framesRead_ = 0;
};

} // namespace sdl
