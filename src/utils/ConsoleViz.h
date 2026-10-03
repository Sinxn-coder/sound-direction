#pragma once

#include <string>

#include "direction/DirectionTypes.h"

namespace sdl {

// One-line ASCII gauge, e.g.  "L [.......#.....|.............] R"
std::string renderDirectionBar(const DirectionResult& r, int width = 31);

// Full status line: time, gauge, direction, delay, confidence.
std::string formatResultLine(double timeSec, const DirectionResult& r);

} // namespace sdl
