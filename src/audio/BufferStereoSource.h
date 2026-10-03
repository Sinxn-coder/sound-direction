#pragma once

#include "audio/AudioSource.h"

namespace sdl {

// Streams pre-loaded left/right buffers block by block.
class BufferStereoSource : public IStereoSource {
public:
    BufferStereoSource(std::vector<float> left, std::vector<float> right, int sampleRate);

    int sampleRate() const override { return sampleRate_; }
    bool read(StereoBlock& out, std::size_t frames) override;

private:
    std::vector<float> left_;
    std::vector<float> right_;
    int sampleRate_;
    std::size_t pos_ = 0;
};

} // namespace sdl
