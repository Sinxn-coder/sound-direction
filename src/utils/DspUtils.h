#pragma once

#include <cstddef>
#include <vector>

namespace sdl::dsp {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSpeedOfSoundMps = 343.0; // dry air, ~20 C

double mean(const float* x, std::size_t n);

// Copies `src` into `dst` with the mean (DC offset) removed.
void removeDc(const float* src, std::size_t n, std::vector<float>& dst);

double energy(const float* x, std::size_t n);
double rms(const float* x, std::size_t n);
inline double rms(const std::vector<float>& x) { return rms(x.data(), x.size()); }

// RMS -> dB relative to full scale (1.0). Floors at -200 dB for silence.
double toDbfs(double rmsValue);

template <typename T>
inline T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

} // namespace sdl::dsp
