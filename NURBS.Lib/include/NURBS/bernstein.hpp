/// @file
/// Bernstein polynomials (A1.2, A1.3).

#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <vector>

namespace NURBS
{

/// Computes a single Bernstein polynomial (Algorithm A1.2, *The NURBS Book*).
///
/// Evaluates \f$B_{i,n}(u) = \frac{n!}{i!\,(n-i)!} u^i (1-u)^{n-i}\f$ with the
/// recurrence \f$B_{i,n}(u) = (1-u) B_{i,n-1}(u) + u B_{i-1,n-1}(u)\f$.
/// @tparam Scalar The floating-point type of @p u and the result.
/// @param i The index of the polynomial.
/// @param n The degree.
/// @param u The parameter value to evaluate at.
/// @return \f$B_{i,n}(u)\f$.
/// @pre `i <= n`.
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

/// Computes all Bernstein polynomials of a given degree
/// (Algorithm A1.3, *The NURBS Book*).
///
/// Evaluates \f$B_{0,n}(u), \ldots, B_{n,n}(u)\f$ together with the recurrence
/// \f$B_{j,k}(u) = (1-u) B_{j,k-1}(u) + u B_{j-1,k-1}(u)\f$, which is cheaper
/// than calling Bernstein() once per index.
/// @tparam Scalar The floating-point type of @p u and the results.
/// @param n The degree.
/// @param u The parameter value to evaluate at.
/// @return A vector `B` of size \f$n + 1\f$ with `B[j]` \f$= B_{j,n}(u)\f$.
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
