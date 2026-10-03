#pragma once

#include "direction/DirectionEngine.h"
#include "direction/DirectionTypes.h"

namespace sdl {

struct SmootherConfig {
    float maxAlpha = 0.6f;   // EMA weight for a frame with confidence 1.0
    float minAlpha = 0.05f;
    int holdFrames = 4;      // keep the last direction this many silent/unknown frames
    float holdDecay = 0.75f; // confidence multiplier per held frame
};

// Smooths per-block results over time so the display does not flicker.
// - Delay is an exponential moving average weighted by each frame's confidence.
// - Brief dropouts (silence between syllables, a bad frame) hold the last estimate.
class TemporalSmoother {
public:
    TemporalSmoother(const DirectionConfig& dirCfg, SmootherConfig cfg = {})
        : dirCfg_(dirCfg), cfg_(cfg) {}

    DirectionResult update(const DirectionResult& raw, int sampleRate);
    void reset();

private:
    DirectionConfig dirCfg_;
    SmootherConfig cfg_;
    bool hasEstimate_ = false;
    int missedFrames_ = 0;
    DirectionResult state_;
};

} // namespace sdl
