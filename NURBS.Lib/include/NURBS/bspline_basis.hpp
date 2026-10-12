/// @file
/// B-spline basis functions and their derivatives (A2.1, A2.2, A2.3, A2.4, A2.5).

#pragma once

#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <ranges>
#include <utility>
#include <vector>

namespace NURBS
{

/// Determines the knot span index (Algorithm A2.1, *The NURBS Book*).
///
/// Finds the index \f$i\f$ with \f$u_i \le u < u_{i+1}\f$ by binary search over
/// the knot vector \f$U = \{u_0, \ldots, u_m\}\f$. The number of basis
/// functions follows from \f$m = n + p + 1\f$, so the book's parameter \f$n\f$
/// is computed from the size of @p U rather than passed in.
///
/// Only the spans \f$p, \ldots, n\f$ make up the parameter domain
/// \f$[u_p, u_{n+1}]\f$, for clamped and unclamped knot vectors alike. At its
/// end, \f$u = u_{n+1}\f$, the span is taken to be \f$n\f$, so the result is
/// always in \f$[p, n]\f$.
/// @tparam Scalar The floating-point type of @p u and the knots.
/// @tparam R A random-access, sized range of knots.
/// @param p The degree.
/// @param u The parameter value to find the span of.
/// @param U The knot vector \f$u_0, \ldots, u_m\f$, non-decreasing.
/// @return The knot span index \f$i\f$.
/// @pre `U.size() >= 2 * p + 2` (at least \f$p + 1\f$ basis functions) and
///      \f$u_p \le u \le u_{n+1}\f$, where \f$n = \f$ `U.size() - p - 2`.
template <std::floating_point Scalar, std::ranges::random_access_range R>
    requires std::ranges::sized_range<R> && std::same_as<std::ranges::range_value_t<R>, Scalar>
[[nodiscard]] constexpr std::size_t FindSpan(std::size_t p, Scalar u, const R& U)
{
    assert(std::ranges::size(U) >= 2 * p + 2);

    const auto n = static_cast<std::size_t>(std::ranges::size(U)) - p - 2;
    const auto first = std::ranges::begin(U);
    using Diff = std::ranges::range_difference_t<R>;
    const auto knot = [first](std::size_t k) { return first[static_cast<Diff>(k)]; };

    assert(knot(p) <= u && u <= knot(n + 1));

    if (u == knot(n + 1)) // Special case
        return n;

    std::size_t low = p;
    std::size_t high = n + 1;
    std::size_t mid = (low + high) / 2;
    while (u < knot(mid) || u >= knot(mid + 1))
    {
        if (u < knot(mid))
            high = mid;
        else
            low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

/// Computes the nonvanishing basis functions (Algorithm A2.2, *The NURBS Book*).
///
/// On the knot span \f$[u_i, u_{i+1})\f$ at most \f$p + 1\f$ of the degree
/// \f$p\f$ basis functions are nonzero, namely
/// \f$N_{i-p,p}(u), \ldots, N_{i,p}(u)\f$. They are computed together from
/// the Cox-de Boor recurrence, sharing intermediate terms and avoiding any
/// division by zero.
/// @tparam Scalar The floating-point type of @p u, the knots and the results.
/// @tparam R A random-access, sized range of knots.
/// @param i The knot span index, as returned by FindSpan().
/// @param u The parameter value to evaluate at.
/// @param p The degree.
/// @param U The knot vector \f$u_0, \ldots, u_m\f$, non-decreasing.
/// @return A vector `N` of size \f$p + 1\f$ with `N[j]` \f$= N_{i-p+j,p}(u)\f$.
/// @pre `U.size() >= 2 * p + 2`, \f$p \le i \le n\f$ and
///      \f$u_i \le u \le u_{i+1}\f$, where \f$n = \f$ `U.size() - p - 2`.
template <std::floating_point Scalar, std::ranges::random_access_range R>
    requires std::ranges::sized_range<R> && std::same_as<std::ranges::range_value_t<R>, Scalar>
[[nodiscard]] constexpr std::vector<Scalar> BasisFuns(std::size_t i, Scalar u, std::size_t p, const R& U)
{
    assert(std::ranges::size(U) >= 2 * p + 2);
    assert(p <= i && i <= static_cast<std::size_t>(std::ranges::size(U)) - p - 2);

    const auto first = std::ranges::begin(U);
    using Diff = std::ranges::range_difference_t<R>;
    const auto knot = [first](std::size_t k) { return first[static_cast<Diff>(k)]; };

    assert(knot(i) <= u && u <= knot(i + 1));

    std::vector<Scalar> N(p + 1, Scalar{0});
    std::vector<Scalar> left(p + 1, Scalar{0});
    std::vector<Scalar> right(p + 1, Scalar{0});
    N[0] = Scalar{1};
    for (std::size_t j = 1; j <= p; ++j)
    {
        left[j] = u - knot(i + 1 - j);
        right[j] = knot(i + j) - u;
        Scalar saved{0};
        for (std::size_t r = 0; r < j; ++r)
        {
            const Scalar temp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
    return N;
}

/// Computes the nonvanishing basis functions and their derivatives
/// (Algorithm A2.3, *The NURBS Book*).
///
/// On the knot span \f$[u_i, u_{i+1})\f$ the nonzero degree \f$p\f$ basis
/// functions are \f$N_{i-p,p}(u), \ldots, N_{i,p}(u)\f$, as in BasisFuns().
/// This also computes their derivatives \f$N^{(k)}_{i-p+j,p}(u)\f$ for
/// \f$k = 1, \ldots, n\f$, from differences of lower-degree basis functions
/// (Eq. 2.10). Each function is a polynomial of degree \f$p\f$ on the span, so
/// every derivative of order \f$k > p\f$ is zero. At a knot, the derivatives
/// are those of the polynomial piece on span @p i.
/// @tparam Scalar The floating-point type of @p u, the knots and the results.
/// @tparam R A random-access, sized range of knots.
/// @param i The knot span index, as returned by FindSpan().
/// @param u The parameter value to evaluate at.
/// @param p The degree.
/// @param n The highest derivative order to compute.
/// @param U The knot vector \f$u_0, \ldots, u_m\f$, non-decreasing.
/// @return A vector `ders` of \f$n + 1\f$ rows, each of size \f$p + 1\f$, with
///         `ders[k][j]` \f$= N^{(k)}_{i-p+j,p}(u)\f$. Row 0 holds the
///         functions themselves, and rows \f$k > p\f$ are zero.
/// @pre `U.size() >= 2 * p + 2`, \f$p \le i \le\f$ `U.size() - p - 2` and
///      \f$u_i \le u \le u_{i+1}\f$.
template <std::floating_point Scalar, std::ranges::random_access_range R>
    requires std::ranges::sized_range<R> && std::same_as<std::ranges::range_value_t<R>, Scalar>
[[nodiscard]] constexpr std::vector<std::vector<Scalar>> DersBasisFuns(std::size_t i, Scalar u, std::size_t p,
                                                                       std::size_t n, const R& U)
{
    assert(std::ranges::size(U) >= 2 * p + 2);
    assert(p <= i && i <= static_cast<std::size_t>(std::ranges::size(U)) - p - 2);

    const auto first = std::ranges::begin(U);
    using Diff = std::ranges::range_difference_t<R>;
    const auto knot = [first](std::size_t k) { return first[static_cast<Diff>(k)]; };

    assert(knot(i) <= u && u <= knot(i + 1));

    // ndu holds the basis functions of every degree up to p in its upper
    // triangle and the knot differences they divide by in its lower triangle.
    std::vector<std::vector<Scalar>> ndu(p + 1, std::vector<Scalar>(p + 1, Scalar{0}));
    std::vector<Scalar> left(p + 1, Scalar{0});
    std::vector<Scalar> right(p + 1, Scalar{0});
    ndu[0][0] = Scalar{1};
    for (std::size_t j = 1; j <= p; ++j)
    {
        left[j] = u - knot(i + 1 - j);
        right[j] = knot(i + j) - u;
        Scalar saved{0};
        for (std::size_t r = 0; r < j; ++r)
        {
            ndu[j][r] = right[r + 1] + left[j - r]; // Lower triangle
            const Scalar temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp; // Upper triangle
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    std::vector<std::vector<Scalar>> ders(n + 1, std::vector<Scalar>(p + 1, Scalar{0}));
    for (std::size_t j = 0; j <= p; ++j)
    {
        ders[0][j] = ndu[j][p];
    }

    // Derivatives above p are zero, so stop at p and leave those rows as they are.
    const std::size_t kmax = std::min(n, p);

    // a holds the coefficients a_{k,j} of Eq. 2.10 in two alternating rows.
    std::vector<std::vector<Scalar>> a(2, std::vector<Scalar>(p + 1, Scalar{0}));
    for (std::size_t r = 0; r <= p; ++r)
    {
        std::size_t s1 = 0;
        std::size_t s2 = 1;
        a[0][0] = Scalar{1};
        // The k-th derivative of N_{i-p+r,p}. The book's rk = r - k can be
        // negative, so indices of the form rk + j are written r + j - k.
        for (std::size_t k = 1; k <= kmax; ++k)
        {
            Scalar d{0};
            const std::size_t pk = p - k;
            if (r >= k)
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][r - k];
                d = a[s2][0] * ndu[r - k][pk];
            }
            const std::size_t j1 = (r + 1 >= k) ? 1 : k - r;
            const std::size_t j2 = (r <= pk + 1) ? k - 1 : p - r;
            for (std::size_t j = j1; j <= j2; ++j)
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][r + j - k];
                d += a[s2][j] * ndu[r + j - k][pk];
            }
            if (r <= pk)
            {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k][r] = d;
            std::swap(s1, s2);
        }
    }

    // Multiply row k by p! / (p - k)!.
    auto factor = static_cast<Scalar>(p);
    for (std::size_t k = 1; k <= kmax; ++k)
    {
        for (std::size_t j = 0; j <= p; ++j)
        {
            ders[k][j] *= factor;
        }
        factor *= static_cast<Scalar>(p - k);
    }
    return ders;
}

/// Computes a single basis function (Algorithm A2.4, *The NURBS Book*).
///
/// Evaluates \f$N_{i,p}(u)\f$ for the basis function index @p i, with no knot
/// span needed. Outside its support \f$[u_i, u_{i+p+1})\f$ the function is zero
/// and is returned at once; otherwise it is built from the degree 0 functions
/// \f$N_{i,0}, \ldots, N_{i+p,0}\f$ by the Cox-de Boor recurrence, skipping
/// terms whose numerator function is zero so that no 0/0 arises.
///
/// At the end of the domain, \f$u = u_{n+1}\f$, span \f$n\f$ is taken to be
/// closed, as in FindSpan(). This differs from the book, which special-cases
/// only \f$N_{0,p}\f$ and \f$N_{n,p}\f$ and so assumes a clamped knot vector.
/// The result therefore always equals the corresponding entry of
/// `BasisFuns(FindSpan(p, u, U), u, p, U)`, or zero if that span does not
/// contain @p i. The cost is \f$O(p^2)\f$, the same as BasisFuns(), which
/// returns all \f$p + 1\f$ nonzero functions; prefer it when more than one is
/// needed.
/// @tparam Scalar The floating-point type of @p u, the knots and the result.
/// @tparam R A random-access, sized range of knots.
/// @param i The basis function index.
/// @param u The parameter value to evaluate at.
/// @param p The degree.
/// @param U The knot vector \f$u_0, \ldots, u_m\f$, non-decreasing.
/// @return \f$N_{i,p}(u)\f$.
/// @pre `U.size() >= 2 * p + 2`, \f$i \le n\f$ and
///      \f$u_p \le u \le u_{n+1}\f$, where \f$n = \f$ `U.size() - p - 2`.
template <std::floating_point Scalar, std::ranges::random_access_range R>
    requires std::ranges::sized_range<R> && std::same_as<std::ranges::range_value_t<R>, Scalar>
[[nodiscard]] constexpr Scalar OneBasisFun(std::size_t i, Scalar u, std::size_t p, const R& U)
{
    assert(std::ranges::size(U) >= 2 * p + 2);

    const auto n = static_cast<std::size_t>(std::ranges::size(U)) - p - 2;
    const auto first = std::ranges::begin(U);
    using Diff = std::ranges::range_difference_t<R>;
    const auto knot = [first](std::size_t k) { return first[static_cast<Diff>(k)]; };

    assert(i <= n);
    assert(knot(p) <= u && u <= knot(n + 1));

    // N_{k,0}(u), with span n closed at the end of the domain.
    const bool atEnd = u == knot(n + 1);
    const auto step = [&](std::size_t k) { return atEnd ? k == n : knot(k) <= u && u < knot(k + 1); };

    // Initialize the degree 0 functions, returning early outside the support.
    std::vector<Scalar> N(p + 1, Scalar{0});
    bool inSupport = false;
    for (std::size_t j = 0; j <= p; ++j)
    {
        if (step(i + j))
        {
            N[j] = Scalar{1};
            inSupport = true;
        }
    }
    if (!inSupport)
        return Scalar{0};

    // Compute the triangular table.
    for (std::size_t k = 1; k <= p; ++k)
    {
        Scalar saved = (N[0] == Scalar{0}) ? Scalar{0} : ((u - knot(i)) * N[0]) / (knot(i + k) - knot(i));
        for (std::size_t j = 0; j < p - k + 1; ++j)
        {
            const Scalar uLeft = knot(i + j + 1);
            const Scalar uRight = knot(i + j + k + 1);
            if (N[j + 1] == Scalar{0})
            {
                N[j] = saved;
                saved = Scalar{0};
            }
            else
            {
                const Scalar temp = N[j + 1] / (uRight - uLeft);
                N[j] = saved + (uRight - u) * temp;
                saved = (u - uLeft) * temp;
            }
        }
    }
    return N[0];
}

/// Computes the derivatives of a single basis function (Algorithm A2.5,
/// *The NURBS Book*).
///
/// Evaluates \f$N^{(k)}_{i,p}(u)\f$ for \f$k = 0, \ldots, n\f$ and the basis
/// function index @p i, with no knot span needed. Outside its support
/// \f$[u_i, u_{i+p+1})\f$ every derivative is zero and is returned at once;
/// otherwise the triangle of functions \f$N_{i+j,q}\f$ is built as in
/// OneBasisFun(), and the \f$k\f$-th derivative is formed from its column
/// \f$q = p - k\f$ by repeated differencing (Eq. 2.9). Terms whose numerator
/// function is zero are skipped, so no 0/0 arises. Each function is a
/// polynomial of degree \f$p\f$ on a span, so every derivative of order
/// \f$k > p\f$ is zero.
///
/// The conventions are those of OneBasisFun(): the derivatives are
/// right-continuous at interior knots, and at the end of the domain,
/// \f$u = u_{n+1}\f$, span \f$n\f$ is taken to be closed. The result therefore
/// always equals column `i - (s - p)` of `DersBasisFuns(s, u, p, n, U)` with
/// `s = FindSpan(p, u, U)`, or zero if span `s` does not contain @p i. The cost
/// is \f$O(p^2 + n^2)\f$; prefer DersBasisFuns() when the derivatives of more
/// than one function are needed.
/// @tparam Scalar The floating-point type of @p u, the knots and the results.
/// @tparam R A random-access, sized range of knots.
/// @param i The basis function index.
/// @param u The parameter value to evaluate at.
/// @param p The degree.
/// @param n The highest derivative order to compute.
/// @param U The knot vector \f$u_0, \ldots, u_m\f$, non-decreasing.
/// @return A vector `ders` of size \f$n + 1\f$ with `ders[k]`
///         \f$= N^{(k)}_{i,p}(u)\f$. Entry 0 is OneBasisFun(), and entries
///         \f$k > p\f$ are zero.
/// @pre `U.size() >= 2 * p + 2`, \f$i \le\f$ `U.size() - p - 2` and
///      \f$u_p \le u \le u_{\text{last}+1}\f$, where \f$\text{last} = \f$
///      `U.size() - p - 2`.
template <std::floating_point Scalar, std::ranges::random_access_range R>
    requires std::ranges::sized_range<R> && std::same_as<std::ranges::range_value_t<R>, Scalar>
[[nodiscard]] constexpr std::vector<Scalar> DersOneBasisFun(std::size_t i, Scalar u, std::size_t p, std::size_t n,
                                                            const R& U)
{
    assert(std::ranges::size(U) >= 2 * p + 2);

    // The book's n, the index of the last basis function; n here is the derivative order.
    const auto last = static_cast<std::size_t>(std::ranges::size(U)) - p - 2;
    const auto first = std::ranges::begin(U);
    using Diff = std::ranges::range_difference_t<R>;
    const auto knot = [first](std::size_t k) { return first[static_cast<Diff>(k)]; };

    assert(i <= last);
    assert(knot(p) <= u && u <= knot(last + 1));

    // N_{k,0}(u), with span last closed at the end of the domain.
    const bool atEnd = u == knot(last + 1);
    const auto step = [&](std::size_t k) { return atEnd ? k == last : knot(k) <= u && u < knot(k + 1); };

    std::vector<Scalar> ders(n + 1, Scalar{0});

    // Initialize the degree 0 functions, returning early outside the support.
    // N[j][q] holds N_{i+j,q}(u).
    std::vector<std::vector<Scalar>> N(p + 1, std::vector<Scalar>(p + 1, Scalar{0}));
    bool inSupport = false;
    for (std::size_t j = 0; j <= p; ++j)
    {
        if (step(i + j))
        {
            N[j][0] = Scalar{1};
            inSupport = true;
        }
    }
    if (!inSupport)
        return ders;

    // Compute the full triangular table, as in OneBasisFun.
    for (std::size_t q = 1; q <= p; ++q)
    {
        Scalar saved =
            (N[0][q - 1] == Scalar{0}) ? Scalar{0} : ((u - knot(i)) * N[0][q - 1]) / (knot(i + q) - knot(i));
        for (std::size_t j = 0; j < p - q + 1; ++j)
        {
            const Scalar uLeft = knot(i + j + 1);
            const Scalar uRight = knot(i + j + q + 1);
            if (N[j + 1][q - 1] == Scalar{0})
            {
                N[j][q] = saved;
                saved = Scalar{0};
            }
            else
            {
                const Scalar temp = N[j + 1][q - 1] / (uRight - uLeft);
                N[j][q] = saved + (uRight - u) * temp;
                saved = (u - uLeft) * temp;
            }
        }
    }
    ders[0] = N[0][p];

    // Derivatives above p are zero, so stop at p and leave those entries as they are.
    const std::size_t kmax = std::min(n, p);

    // The k-th derivative, from the degree p - k functions N_{i,p-k}, ..., N_{i+k,p-k}.
    std::vector<Scalar> ND(p + 1, Scalar{0});
    for (std::size_t k = 1; k <= kmax; ++k)
    {
        for (std::size_t j = 0; j <= k; ++j)
        {
            ND[j] = N[j][p - k];
        }
        // Pass jj turns ND[j] into the (jj)-th derivative of N_{i+j,d}.
        for (std::size_t jj = 1; jj <= k; ++jj)
        {
            const std::size_t d = p - k + jj;
            const auto degree = static_cast<Scalar>(d);
            Scalar saved = (ND[0] == Scalar{0}) ? Scalar{0} : ND[0] / (knot(i + d) - knot(i));
            for (std::size_t j = 0; j < k - jj + 1; ++j)
            {
                // The book prints U[i+j+p+jj+1]; the right end of N_{i+j+1,d-1}'s
                // support is u_{i+j+1+d}.
                const Scalar uLeft = knot(i + j + 1);
                const Scalar uRight = knot(i + j + d + 1);
                if (ND[j + 1] == Scalar{0})
                {
                    ND[j] = degree * saved;
                    saved = Scalar{0};
                }
                else
                {
                    const Scalar temp = ND[j + 1] / (uRight - uLeft);
                    ND[j] = degree * (saved - temp);
                    saved = temp;
                }
            }
        }
        ders[k] = ND[0];
    }
    return ders;
}

} // namespace NURBS
