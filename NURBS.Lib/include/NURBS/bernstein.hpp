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
[[nodiscard]] constexpr Scalar Bernstein(std::size_t i, std::size_t n, Scalar u)
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

// Algorithm A1.3 (The NURBS Book): compute all n+1 Bernstein polynomials
// B_{0,n}(u), ..., B_{n,n}(u) of degree n at u, using the recurrence
// B_{j,k}(u) = (1-u) * B_{j,k-1}(u) + u * B_{j-1,k-1}(u).
// Returns a vector B of size n + 1 with B[j] = B_{j,n}(u).
template <std::floating_point Scalar>
[[nodiscard]] constexpr std::vector<Scalar> AllBernstein(std::size_t n, Scalar u)
{
    std::vector<Scalar> B(n + 1, Scalar{0});
    B[0] = Scalar{1};
    const Scalar u1 = Scalar{1} - u;
    for (std::size_t j = 1; j <= n; ++j)
    {
        Scalar saved{0};
        for (std::size_t k = 0; k < j; ++k)
        {
            const Scalar temp = B[k];
            B[k] = saved + u1 * temp;
            saved = u * temp;
        }
        B[j] = saved;
    }
    return B;
}

} // namespace NURBS
