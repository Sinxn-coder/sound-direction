#pragma once

#include <cstdint>
#include <limits>
#include <vector>

namespace sdl {

struct SimulationParams {
    // Time difference of arrival, same sign convention as DirectionResult:
    //   > 0 : source on the RIGHT (left mic receives the sound later)
    //   < 0 : source on the LEFT  (right mic receives the sound later)
    // Fractional values are supported (windowed-sinc fractional delay).
    double tdoaSamples = 0.0;

    // Head shadow: the far-side mic is attenuated by this many dB.
    float farEarAttenuationDb = 0.0f;

    // Independent white noise added to each mic. Infinity = no noise.
    float snrDb = std::numeric_limits<float>::infinity();
    uint32_t noiseSeed = 1234;
};

struct SimulatedStereo {
    std::vector<float> left;
    std::vector<float> right;
};

// Creates a two-microphone recording from a mono source.
SimulatedStereo simulateTwoMics(const std::vector<float>& mono, const SimulationParams& params);

// Delays a signal by a (possibly fractional) number of samples >= 0. Output has
// the same length as the input; the beginning is zero-filled.
std::vector<float> fractionalDelay(const std::vector<float>& x, double delaySamples);

// Rough spherical-head model for a source at `azimuthDeg` (-90 left .. +90 right):
//   ITD = (r / c) * (theta + sin(theta))  (Woodworth), r = spacing / 2
//   ILD ~ 10 dB * |sin(theta)|            (broadband head-shadow approximation)
SimulationParams paramsFromAzimuth(double azimuthDeg, double micSpacingMeters,
                                   double speedOfSoundMps, int sampleRate);

} // namespace sdl
