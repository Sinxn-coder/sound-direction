#include "direction/CrossCorrelation.h"

#include <algorithm>
#include <cmath>

namespace sdl {
namespace {
// Accumulator for the O(N * lags) dot-product loop. The ESP32-S3 FPU is single
// precision only (double is emulated in software), so embedded builds define
// SDL_CORE_FLOAT_ACCUM. Desktop builds keep double.
#ifdef SDL_CORE_FLOAT_ACCUM
using DotAccum = float;
#else
using DotAccum = double;
#endif
} // namespace

const std::vector<float>& CrossCorrelator::compute(const float* a, const float* b, std::size_t n,
                                                   int maxLag) {
    const int numLags = 2 * maxLag + 1;
    ncc_.assign(static_cast<std::size_t>(numLags), 0.0f);
    if (maxLag < 0 || n <= static_cast<std::size_t>(2 * maxLag)) return ncc_;

    // Prefix sums of squares let us get the energy of any overlap window in O(1).
    prefixA_.resize(n + 1);
    prefixB_.resize(n + 1);
    prefixA_[0] = prefixB_[0] = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        prefixA_[i + 1] = prefixA_[i] + static_cast<double>(a[i]) * a[i];
        prefixB_[i + 1] = prefixB_[i] + static_cast<double>(b[i]) * b[i];
    }

    const long long N = static_cast<long long>(n);
    for (int k = -maxLag; k <= maxLag; ++k) {
        // Overlap: n in [lo, hi) such that both n and n+k are valid indices.
        const long long lo = std::max(0LL, -static_cast<long long>(k));
        const long long hi = std::min(N, N - k);

        DotAccum dot = 0;
        const float* pa = a + lo;
        const float* pb = b + lo + k;
        const std::size_t len = static_cast<std::size_t>(hi - lo);
        for (std::size_t i = 0; i < len; ++i) dot += static_cast<DotAccum>(pa[i]) * pb[i];

        const double ea = prefixA_[hi] - prefixA_[lo];
        const double eb = prefixB_[hi + k] - prefixB_[lo + k];
        const double denom = std::sqrt(ea * eb);
        ncc_[static_cast<std::size_t>(k + maxLag)] =
            denom > 1e-20 ? static_cast<float>(dot / denom) : 0.0f;
    }
    return ncc_;
}

CorrelationPeak CrossCorrelator::findPeak(const std::vector<float>& ncc, int maxLag) {
    CorrelationPeak p;
    const int size = static_cast<int>(ncc.size());
    if (size == 0) return p;

    int best = 0;
    for (int i = 1; i < size; ++i)
        if (ncc[i] > ncc[best]) best = i;

    p.index = best;
    p.lag = best - maxLag;
    p.value = ncc[best];
    p.atEdge = (best == 0 || best == size - 1);

    // Sub-sample refinement: fit a parabola through the peak and its neighbours.
    p.refinedLag = static_cast<double>(p.lag);
    if (!p.atEdge) {
        const double ym1 = ncc[best - 1], y0 = ncc[best], yp1 = ncc[best + 1];
        const double denom = ym1 - 2.0 * y0 + yp1;
        if (std::fabs(denom) > 1e-12) {
            double offset = 0.5 * (ym1 - yp1) / denom;
            offset = std::max(-0.5, std::min(0.5, offset));
            p.refinedLag += offset;
        }
    }

    // Strongest competing local maximum. The boundaries count as local maxima when
    // the curve is still rising into them (i.e. a peak may lie outside the range).
    for (int i = 0; i < size; ++i) {
        if (i == best) continue;
        const bool geLeft = (i == 0) || ncc[i] >= ncc[i - 1];
        const bool geRight = (i == size - 1) || ncc[i] >= ncc[i + 1];
        if (geLeft && geRight && ncc[i] > p.secondValue) p.secondValue = ncc[i];
    }
    return p;
}

} // namespace sdl
