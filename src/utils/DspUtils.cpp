#include "utils/DspUtils.h"

#include <cmath>

namespace sdl::dsp {

double mean(const float* x, std::size_t n) {
    if (n == 0) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 0; i < n; ++i) sum += x[i];
    return sum / static_cast<double>(n);
}

void removeDc(const float* src, std::size_t n, std::vector<float>& dst) {
    dst.resize(n);
    const float m = static_cast<float>(mean(src, n));
    for (std::size_t i = 0; i < n; ++i) dst[i] = src[i] - m;
}

double energy(const float* x, std::size_t n) {
    double e = 0.0;
    for (std::size_t i = 0; i < n; ++i) e += static_cast<double>(x[i]) * x[i];
    return e;
}

double rms(const float* x, std::size_t n) {
    if (n == 0) return 0.0;
    return std::sqrt(energy(x, n) / static_cast<double>(n));
}

double toDbfs(double rmsValue) {
    if (rmsValue <= 1e-10) return -200.0;
    return 20.0 * std::log10(rmsValue);
}

} // namespace sdl::dsp
