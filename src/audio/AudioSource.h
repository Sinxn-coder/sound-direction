#pragma once

#include <cstddef>
#include <vector>

namespace sdl {

// A block of synchronized two-microphone audio, float samples in [-1, 1].
struct StereoBlock {
    std::vector<float> left;
    std::vector<float> right;
    int sampleRate = 0;
    double startTimeSec = 0.0;

    std::size_t frames() const { return left.size(); }
};

// Audio input abstraction. The direction engine never talks to a device or a file
// directly; it only consumes StereoBlocks produced by one of these.
//
// Implementations:
//   - BufferStereoSource : in-memory buffers (WAV files, simulation)      [now]
//   - Esp32I2sSource     : two I2S MEMS mics on the ESP32-S3 (or a serial /
//                          UDP stream from it into the laptop)              [later]
class IStereoSource {
public:
    virtual ~IStereoSource() = default;

    virtual int sampleRate() const = 0;

    // Fills `out` with up to `frames` frames. Returns false when the stream ended.
    virtual bool read(StereoBlock& out, std::size_t frames) = 0;
};

} // namespace sdl
