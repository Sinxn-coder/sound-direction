#pragma once

namespace sdl {

enum class Direction {
    Unknown, // not enough evidence (low confidence, bad input)
    Silent,  // input below the energy threshold
    Center,  // roughly equal arrival time: FRONT or BACK (2 mics cannot tell which)
    Left,
    Right,
    // Reserved for later milestones; the current engine never outputs these.
    Front,
    Back,
    Up,
    Down,
};

inline const char* toString(Direction d) {
    switch (d) {
        case Direction::Unknown: return "UNKNOWN";
        case Direction::Silent:  return "SILENT";
        case Direction::Center:  return "CENTER";
        case Direction::Left:    return "LEFT";
        case Direction::Right:   return "RIGHT";
        case Direction::Front:   return "FRONT";
        case Direction::Back:    return "BACK";
        case Direction::Up:      return "UP";
        case Direction::Down:    return "DOWN";
    }
    return "?";
}

// Sign convention used everywhere in this project: POSITIVE = towards the RIGHT.
//   delaySamples = t_arrival(left mic) - t_arrival(right mic)
//   > 0  -> right mic heard it first -> sound is on the RIGHT
//   < 0  -> left mic heard it first  -> sound is on the LEFT
struct DirectionResult {
    Direction direction = Direction::Unknown;
    float confidence = 0.0f;       // 0..1

    double delaySamples = 0.0;     // sub-sample (parabolic interpolation)
    double delayMs = 0.0;

    double azimuthDeg = 0.0;       // -90 = full left, 0 = center, +90 = full right (free-field estimate)

    // Diagnostics
    float peakCorrelation = 0.0f;  // normalized cross-correlation at the peak, -1..1
    float levelDiffDb = 0.0f;      // right level - left level (positive = right louder)
    float levelDbfs = -200.0f;     // level of the louder channel
    int maxLagSamples = 0;         // search range used: [-maxLag, +maxLag]
    bool peakAtSearchEdge = false; // true delay may be outside the physical range
};

} // namespace sdl
