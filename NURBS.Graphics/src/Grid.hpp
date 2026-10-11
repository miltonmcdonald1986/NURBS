#pragma once

#include <cmath>

namespace Graphics::Detail
{

// The smallest power of ten that is at least x, for grid spacing.
// Precondition: x > 0.
[[nodiscard]] inline double PowerOfTenAtLeast(double x)
{
    return std::pow(10.0, std::ceil(std::log10(x)));
}

} // namespace Graphics::Detail
