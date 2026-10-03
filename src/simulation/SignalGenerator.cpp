#include "simulation/SignalGenerator.h"

#include <algorithm>
#include <cmath>
#include <random>

#include "utils/DspUtils.h"

namespace sdl {

std::vector<float> generateNoiseBursts(int sampleRate, double seconds, uint32_t seed, double burstSec,
                                       double gapSec, float amplitude) {
    const std::size_t n = static_cast<std::size_t>(seconds * sampleRate);
    std::vector<float> out(n, 0.0f);
    std::mt19937 rng(seed);
    std::normal_distribution<float> dist(0.0f, 1.0f);

    const std::size_t burst = static_cast<std::size_t>(burstSec * sampleRate);
    const std::size_t period = burst + static_cast<std::size_t>(gapSec * sampleRate);
    const std::size_t fade = std::max<std::size_t>(1, static_cast<std::size_t>(0.005 * sampleRate));

    // One-pole low-pass (~6 kHz) + DC-blocking high-pass (~100 Hz) for a less harsh sound.
    const float lpA = static_cast<float>(std::exp(-2.0 * dsp::kPi * 6000.0 / sampleRate));
    const float hpA = static_cast<float>(std::exp(-2.0 * dsp::kPi * 100.0 / sampleRate));
    float lp = 0.0f, hpPrevIn = 0.0f, hpPrevOut = 0.0f;

    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t posInPeriod = i % period;
        if (posInPeriod >= burst) continue;

        float env = 1.0f;
        if (posInPeriod < fade) env = static_cast<float>(posInPeriod) / fade;
        else if (burst - posInPeriod < fade) env = static_cast<float>(burst - posInPeriod) / fade;

        lp = (1.0f - lpA) * dist(rng) + lpA * lp;
        const float hp = hpA * (hpPrevOut + lp - hpPrevIn);
        hpPrevIn = lp;
        hpPrevOut = hp;
        out[i] = hp * env;
    }

    // Normalize the peak to `amplitude`.
    float peak = 0.0f;
    for (float v : out) peak = std::max(peak, std::fabs(v));
    if (peak > 0.0f)
        for (float& v : out) v *= amplitude / peak;
    return out;
}

std::vector<float> generateTone(int sampleRate, double seconds, double freqHz, float amplitude) {
    const std::size_t n = static_cast<std::size_t>(seconds * sampleRate);
    std::vector<float> out(n);
    for (std::size_t i = 0; i < n; ++i)
        out[i] = amplitude * static_cast<float>(std::sin(2.0 * dsp::kPi * freqHz * i / sampleRate));
    return out;
}

} // namespace sdl
