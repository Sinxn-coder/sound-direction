#pragma once

#include <vector>

#include "direction/CrossCorrelation.h"
#include "direction/DirectionTypes.h"

namespace sdl {

struct DirectionConfig {
    double micSpacingMeters = 0.18;
    double speedOfSoundMps = 343.0;

    // Mics on either side of a head: sound has to travel *around* the head, so the
    // real maximum delay is ~30% larger than spacing / c (Woodworth model gives
    // 1.29x at 90 degrees). The search range is widened by this factor.
    double searchMarginFactor = 1.35;
    int extraLagSamples = 1;

    float silenceThresholdDbfs = -55.0f; // louder channel below this -> SILENT
    float minConfidence = 0.30f;         // below this -> UNKNOWN
    double centerDeadzoneDeg = 8.0;      // |azimuth| below this -> CENTER

    // Largest lag (in samples) that is physically plausible for this geometry.
    int maxLagSamples(int sampleRate) const;
};

// Free-field azimuth estimate from a time difference (seconds). Clamped to +-90.
double delayToAzimuthDeg(double delaySeconds, const DirectionConfig& cfg);

// Maps an azimuth + confidence to a coarse direction (LEFT / RIGHT / CENTER / UNKNOWN).
Direction classifyDirection(double azimuthDeg, float confidence, const DirectionConfig& cfg);

// The core localization engine. Platform independent: takes two synchronized float
// buffers (same length, same clock) and returns a direction estimate.
// The same class runs on the laptop (WAV / simulation) and later on the ESP32-S3.
class DirectionEngine {
public:
    explicit DirectionEngine(DirectionConfig cfg = {}) : cfg_(cfg) {}

    DirectionResult process(const std::vector<float>& left, const std::vector<float>& right,
                            int sampleRate);

    const DirectionConfig& config() const { return cfg_; }
    void setConfig(const DirectionConfig& cfg) { cfg_ = cfg; }

    // Last correlation curve (lag i - maxLag), handy for debugging / plotting.
    const std::vector<float>& lastCorrelation() const { return lastNcc_; }

private:
    DirectionConfig cfg_;
    CrossCorrelator correlator_;
    std::vector<float> left_;   // DC-removed scratch copies
    std::vector<float> right_;
    std::vector<float> lastNcc_;
};

} // namespace sdl
