#pragma once

#include <string>
#include <vector>

namespace sdl {

struct WavData {
    int sampleRate = 0;
    int channels = 0;
    std::vector<std::vector<float>> channelData; // [channel][frame], floats in [-1, 1]

    std::size_t frames() const { return channelData.empty() ? 0 : channelData[0].size(); }
};

// Reads PCM 8/16/24/32-bit integer and 32/64-bit float WAV files
// (including WAVE_FORMAT_EXTENSIBLE). Returns false and sets `error` on failure.
bool readWav(const std::string& path, WavData& out, std::string& error);

// Writes 16-bit PCM. All channels must have the same length.
bool writeWav(const std::string& path, const std::vector<std::vector<float>>& channels,
              int sampleRate, std::string& error);

// Averages all channels into one.
std::vector<float> downmixToMono(const WavData& wav);

} // namespace sdl
