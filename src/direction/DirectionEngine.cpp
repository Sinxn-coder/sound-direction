#include "direction/DirectionEngine.h"

#include <algorithm>
#include <cmath>

#include "utils/DspUtils.h"

namespace sdl {

int DirectionConfig::maxLagSamples(int sampleRate) const {
    const double maxDelaySec = micSpacingMeters * searchMarginFactor / speedOfSoundMps;
    return static_cast<int>(std::ceil(maxDelaySec * sampleRate)) + extraLagSamples;
}

double delayToAzimuthDeg(double delaySeconds, const DirectionConfig& cfg) {
    const double s = dsp::clamp(delaySeconds * cfg.speedOfSoundMps / cfg.micSpacingMeters, -1.0, 1.0);
    return std::asin(s) * 180.0 / dsp::kPi;
}

Direction classifyDirection(double azimuthDeg, float confidence, const DirectionConfig& cfg) {
    if (confidence < cfg.minConfidence) return Direction::Unknown;
    if (std::fabs(azimuthDeg) < cfg.centerDeadzoneDeg) return Direction::Center;
    return azimuthDeg > 0.0 ? Direction::Right : Direction::Left;
}

DirectionResult DirectionEngine::process(const std::vector<float>& left,
                                         const std::vector<float>& right, int sampleRate) {
    DirectionResult r;
    if (sampleRate <= 0 || left.empty() || left.size() != right.size()) return r; // UNKNOWN

    const std::size_t n = left.size();
    const int maxLag = cfg_.maxLagSamples(sampleRate);
    r.maxLagSamples = maxLag;
    // Need clearly more samples than the lag range for a meaningful correlation.
    if (n < static_cast<std::size_t>(4 * maxLag + 1)) return r;

    // 1. DC removal (MEMS mics / ADCs often have an offset that would bias the NCC).
    dsp::removeDc(left.data(), n, left_);
    dsp::removeDc(right.data(), n, right_);

    // 2. Energy / silence gate.
    const double dbL = dsp::toDbfs(dsp::rms(left_));
    const double dbR = dsp::toDbfs(dsp::rms(right_));
    r.levelDbfs = static_cast<float>(std::max(dbL, dbR));
    r.levelDiffDb = static_cast<float>(dbR - dbL);
    if (r.levelDbfs < cfg_.silenceThresholdDbfs) {
        r.direction = Direction::Silent;
        return r;
    }

    // 3. Normalized cross-correlation, restricted to physically possible lags.
    //    ncc(k) = sum right[n] * left[n+k]  -> peak at k > 0 means LEFT is late,
    //    i.e. the sound reached the RIGHT mic first (positive = right convention).
    lastNcc_ = correlator_.compute(right_.data(), left_.data(), n, maxLag);
    const CorrelationPeak peak = CrossCorrelator::findPeak(lastNcc_, maxLag);

    r.peakCorrelation = peak.value;
    r.peakAtSearchEdge = peak.atEdge;
    r.delaySamples = peak.refinedLag;
    r.delayMs = peak.refinedLag * 1000.0 / sampleRate;
    r.azimuthDeg = delayToAzimuthDeg(peak.refinedLag / sampleRate, cfg_);

    // 4. Confidence: product of independent sanity factors, each in [0, 1].
    float conf = 0.0f;
    if (peak.value > 0.0f) {
        // How similar the two channels are once aligned.
        const float cPeak = dsp::clamp((peak.value - 0.2f) / 0.7f, 0.0f, 1.0f);
        // How unambiguous the peak is (periodic sounds produce competing peaks).
        const float second = std::max(0.0f, peak.secondValue);
        const float cUnique = dsp::clamp((1.0f - second / peak.value) / 0.5f, 0.0f, 1.0f);
        // How far above the silence threshold we are.
        const float cEnergy =
            dsp::clamp((r.levelDbfs - cfg_.silenceThresholdDbfs) / 15.0f, 0.0f, 1.0f);
        // A peak at the edge of the search range means the "true" delay is probably
        // outside what the geometry allows: reflection, noise, or bad wiring.
        const float cEdge = peak.atEdge ? 0.3f : 1.0f;
        // Level difference (head shadow) disagreeing with timing is suspicious.
        float cLevel = 1.0f;
        if (std::fabs(r.azimuthDeg) >= cfg_.centerDeadzoneDeg && std::fabs(r.levelDiffDb) > 1.5f &&
            (r.levelDiffDb > 0.0f) != (r.delaySamples > 0.0)) {
            cLevel = 0.7f;
        }
        conf = cPeak * (0.4f + 0.6f * cUnique) * cEnergy * cEdge * cLevel;
    }
    r.confidence = conf;

    // 5. Coarse direction.
    r.direction = classifyDirection(r.azimuthDeg, r.confidence, cfg_);
    return r;
}

} // namespace sdl
