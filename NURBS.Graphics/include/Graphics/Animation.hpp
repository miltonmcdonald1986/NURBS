#pragma once

#include <cmath>

namespace Graphics
{

// Moves value forward by fraction of [min, max], wrapping from max back to min.
inline void AdvanceWrapping(double& value, double min, double max, double fraction)
{
    const double range = max - min;
    value = min + std::fmod(value - min + range * fraction, range);
}

// Moves value by fraction of [min, max] in the direction of direction's sign,
// bouncing off either end, where direction flips.
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
