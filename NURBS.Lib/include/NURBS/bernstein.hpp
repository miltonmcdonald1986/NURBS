#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <vector>

namespace NURBS
{

// Algorithm A1.2 (The NURBS Book): compute the value of the Bernstein
// polynomial B_{i,n}(u) = n! / (i! (n-i)!) * u^i * (1-u)^(n-i) using the
// recurrence B_{i,n}(u) = (1-u) * B_{i,n-1}(u) + u * B_{i-1,n-1}(u).
// Precondition: i <= n.
template <std::floating_point Scalar>
[[nodiscard]] constexpr Scalar bernstein(std::size_t i, std::size_t n, Scalar u)
{
    assert(i <= n);

    std::vector<Scalar> temp(n + 1, Scalar{0});
    temp[n - i] = Scalar{1};
    const Scalar u1 = Scalar{1} - u;
    for (std::size_t k = 1; k <= n; ++k)
        for (auto j = n; j >= k; --j)
            temp[j] = u1 * temp[j] + u * temp[j - 1];
    return temp[n];
}

} // namespace NURBS
