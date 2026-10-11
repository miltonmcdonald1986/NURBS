/// @file
/// Helpers that step an animated value through a range.

#pragma once

#include <cmath>

namespace Graphics
{

/// Moves @p value forward by @p fraction of [@p min, @p max], wrapping from
/// @p max back to @p min.
/// @param value The value to advance, updated in place.
/// @param min The start of the range.
/// @param max The end of the range.
/// @param fraction The share of the range to move by.
/// @pre @p max > @p min.
inline void AdvanceWrapping(double& value, double min, double max, double fraction)
{
    const double range = max - min;
    value = min + std::fmod(value - min + range * fraction, range);
}

/// Moves @p value by @p fraction of [@p min, @p max] in the direction of
/// @p direction's sign, bouncing off either end, where @p direction flips.
/// @param value The value to advance, updated in place.
/// @param direction The direction of travel, 1 or -1, flipped in place on a bounce.
/// @param min The start of the range.
/// @param max The end of the range.
/// @param fraction The share of the range to move by.
inline void AdvanceBouncing(double& value, double& direction, double min, double max, double fraction)
{
    value += direction * (max - min) * fraction;
    if (value > max)
    {
        value = 2.0 * max - value;
        direction = -1.0;
    }
    else if (value < min)
    {
        value = 2.0 * min - value;
        direction = 1.0;
    }
}

} // namespace Graphics
