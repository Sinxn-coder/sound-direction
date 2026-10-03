#include "direction/TemporalSmoother.h"

#include "utils/DspUtils.h"

namespace sdl {

void TemporalSmoother::reset() {
    hasEstimate_ = false;
    missedFrames_ = 0;
    state_ = DirectionResult{};
}

DirectionResult TemporalSmoother::update(const DirectionResult& raw, int sampleRate) {
    const bool valid = raw.direction == Direction::Left || raw.direction == Direction::Right ||
                       raw.direction == Direction::Center;

    if (!valid) {
        ++missedFrames_;
        if (!hasEstimate_ || missedFrames_ > cfg_.holdFrames) {
            hasEstimate_ = false;
            state_ = raw;
            return raw;
        }
        // Hold the previous estimate with decaying confidence.
        state_.confidence *= cfg_.holdDecay;
        state_.direction = classifyDirection(state_.azimuthDeg, state_.confidence, dirCfg_);
        return state_;
    }

    missedFrames_ = 0;
    if (!hasEstimate_) {
        state_ = raw;
        hasEstimate_ = true;
        return state_;
    }

    const float alpha = dsp::clamp(cfg_.maxAlpha * raw.confidence, cfg_.minAlpha, cfg_.maxAlpha);
    const double delay = state_.delaySamples + alpha * (raw.delaySamples - state_.delaySamples);
    const float conf = state_.confidence + alpha * (raw.confidence - state_.confidence);

    state_ = raw; // copy diagnostics from the latest frame
    state_.delaySamples = delay;
    state_.delayMs = delay * 1000.0 / sampleRate;
    state_.azimuthDeg = delayToAzimuthDeg(delay / sampleRate, dirCfg_);
    state_.confidence = conf;
    state_.direction = classifyDirection(state_.azimuthDeg, state_.confidence, dirCfg_);
    return state_;
}

} // namespace sdl
