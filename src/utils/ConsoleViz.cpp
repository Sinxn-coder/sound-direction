#include "utils/ConsoleViz.h"

#include <cmath>
#include <cstdio>

namespace sdl {

std::string renderDirectionBar(const DirectionResult& r, int width) {
    if (width < 5) width = 5;
    if (width % 2 == 0) ++width; // odd so there is a true center
    std::string bar(static_cast<std::size_t>(width), '.');
    const int center = width / 2;
    bar[center] = '|';

    const bool hasEstimate = r.direction != Direction::Silent && r.direction != Direction::Unknown;
    if (hasEstimate) {
        const double t = (r.azimuthDeg + 90.0) / 180.0; // 0 = left, 1 = right
        int idx = static_cast<int>(std::lround(t * (width - 1)));
        idx = idx < 0 ? 0 : (idx >= width ? width - 1 : idx);
        bar[idx] = r.confidence >= 0.6f ? '#' : '+';
    }
    return "L [" + bar + "] R";
}

std::string formatResultLine(double timeSec, const DirectionResult& r) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%7.2fs  %s  %-7s delay %+7.2f smp (%+6.3f ms)  az %+5.0f deg  conf %.2f",
                  timeSec, renderDirectionBar(r).c_str(), toString(r.direction), r.delaySamples,
                  r.delayMs, r.azimuthDeg, r.confidence);
    return buf;
}

} // namespace sdl
