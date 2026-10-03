#pragma once

#include <cstdint>
#include <vector>

namespace sdl {

// Broadband test signal: bursts of band-limited noise separated by silence.
// Broadband sounds (claps, knocks, speech consonants, alarms with harmonics) are
// what TDOA works best on; pure tones are ambiguous (see README).
std::vector<float> generateNoiseBursts(int sampleRate, double seconds, uint32_t seed = 42,
                                       double burstSec = 0.30, double gapSec = 0.20,
                                       float amplitude = 0.3f);

// Plain sine tone, useful for demonstrating the periodic-ambiguity limitation.
std::vector<float> generateTone(int sampleRate, double seconds, double freqHz, float amplitude = 0.3f);

} // namespace sdl
