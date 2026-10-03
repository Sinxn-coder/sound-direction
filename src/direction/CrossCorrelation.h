#pragma once

#include <cstddef>
#include <vector>

namespace sdl {

struct CorrelationPeak {
    int index = 0;            // index into the correlation vector
    int lag = 0;              // integer lag = index - maxLag
    double refinedLag = 0.0;  // sub-sample lag from parabolic interpolation
    float value = 0.0f;       // NCC value at the peak
    float secondValue = -1.0f;// best competing local maximum (-1 if none)
    bool atEdge = false;      // peak sits on the boundary of the search range
};

// Normalized cross-correlation over a limited lag range.
//
//   ncc(k) = sum_n a[n] * b[n+k] / sqrt( sum_n a[n]^2 * sum_n b[n+k]^2 )
//
// where the sums run only over the samples that overlap for lag k. Values are
// in [-1, 1] and independent of the overall gain of either channel.
//
// Holds its own scratch buffers so repeated calls do not allocate (important on
// the ESP32 where we process a block every few tens of milliseconds).
class CrossCorrelator {
public:
    // Returns a vector of size 2*maxLag+1; element i corresponds to lag i - maxLag.
    // Inputs should already be DC-free. Requires n > 2*maxLag.
    const std::vector<float>& compute(const float* a, const float* b, std::size_t n, int maxLag);

    static CorrelationPeak findPeak(const std::vector<float>& ncc, int maxLag);

private:
    std::vector<double> prefixA_;
    std::vector<double> prefixB_;
    std::vector<float> ncc_;
};

} // namespace sdl
