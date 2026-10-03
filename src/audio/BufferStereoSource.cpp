#include "audio/BufferStereoSource.h"

#include <algorithm>

namespace sdl {

BufferStereoSource::BufferStereoSource(std::vector<float> left, std::vector<float> right,
                                       int sampleRate)
    : left_(std::move(left)), right_(std::move(right)), sampleRate_(sampleRate) {
    const std::size_t n = std::min(left_.size(), right_.size());
    left_.resize(n);
    right_.resize(n);
}

bool BufferStereoSource::read(StereoBlock& out, std::size_t frames) {
    if (pos_ >= left_.size() || frames == 0) return false;
    const std::size_t n = std::min(frames, left_.size() - pos_);
    out.left.assign(left_.begin() + pos_, left_.begin() + pos_ + n);
    out.right.assign(right_.begin() + pos_, right_.begin() + pos_ + n);
    out.sampleRate = sampleRate_;
    out.startTimeSec = static_cast<double>(pos_) / sampleRate_;
    pos_ += n;
    return true;
}

} // namespace sdl
