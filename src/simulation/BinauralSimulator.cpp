#include "simulation/BinauralSimulator.h"

#include <cmath>
#include <random>

#include "utils/DspUtils.h"

namespace sdl {

std::vector<float> fractionalDelay(const std::vector<float>& x, double delaySamples) {
    const long long n = static_cast<long long>(x.size());
    std::vector<float> y(x.size(), 0.0f);
    if (delaySamples < 0.0) delaySamples = 0.0;

    const double intPart = std::floor(delaySamples);
    const double frac = delaySamples - intPart;
    const long long d = static_cast<long long>(intPart);

    if (frac < 1e-9) { // exact integer shift
        for (long long i = d; i < n; ++i) y[i] = x[i - d];
        return y;
    }

    // Windowed-sinc interpolation (Blackman window, 2*H taps).
    constexpr int H = 16;
    for (long long i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) - delaySamples; // read position in x
        const long long base = static_cast<long long>(std::floor(t));
        double acc = 0.0;
        for (long long j = base - H + 1; j <= base + H; ++j) {
            if (j < 0 || j >= n) continue;
            const double u = t - static_cast<double>(j); // in (-H, H]
            const double sinc = std::fabs(u) < 1e-12 ? 1.0 : std::sin(dsp::kPi * u) / (dsp::kPi * u);
            const double w = 0.42 + 0.5 * std::cos(dsp::kPi * u / H) + 0.08 * std::cos(2.0 * dsp::kPi * u / H);
            acc += x[j] * sinc * w;
        }
        y[i] = static_cast<float>(acc);
    }
    return y;
}

SimulatedStereo simulateTwoMics(const std::vector<float>& mono, const SimulationParams& p) {
    SimulatedStereo s;
    const float farGain = static_cast<float>(std::pow(10.0, -std::fabs(p.farEarAttenuationDb) / 20.0));

    if (p.tdoaSamples > 0.0) { // source on the right: left is the far ear
        s.right = mono;
        s.left = fractionalDelay(mono, p.tdoaSamples);
        for (float& v : s.left) v *= farGain;
    } else if (p.tdoaSamples < 0.0) { // source on the left: right is the far ear
        s.left = mono;
        s.right = fractionalDelay(mono, -p.tdoaSamples);
        for (float& v : s.right) v *= farGain;
    } else {
        s.left = mono;
        s.right = mono;
    }

    if (std::isfinite(p.snrDb)) {
        const double signalRms = dsp::rms(mono);
        const double noiseRms = signalRms / std::pow(10.0, p.snrDb / 20.0);
        std::mt19937 rng(p.noiseSeed);
        std::normal_distribution<float> dist(0.0f, static_cast<float>(noiseRms));
        for (float& v : s.left) v += dist(rng);
        for (float& v : s.right) v += dist(rng);
    }
    return s;
}

SimulationParams paramsFromAzimuth(double azimuthDeg, double micSpacingMeters,
                                   double speedOfSoundMps, int sampleRate) {
    const double theta = dsp::clamp(azimuthDeg, -90.0, 90.0) * dsp::kPi / 180.0;
    const double radius = micSpacingMeters / 2.0;
    const double itdSec = (radius / speedOfSoundMps) * (theta + std::sin(theta));

    SimulationParams p;
    p.tdoaSamples = itdSec * sampleRate;
    p.farEarAttenuationDb = static_cast<float>(10.0 * std::fabs(std::sin(theta)));
    return p;
}

} // namespace sdl
