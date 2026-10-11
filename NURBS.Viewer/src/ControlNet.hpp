#pragma once

#include <glm/vec3.hpp>

#include <cstddef>
#include <ranges>
#include <span>
#include <vector>

namespace NURBS::Viewer
{

// Shared by the scenes whose surfaces are defined by a control net P[i][j],
// i = 0..n, j = 0..m (Bezier and rational Bezier surfaces). The scenes edit a
// copy of the net stored flat in row-major order, P[i][j] at i * (m + 1) + j.

// Copies a range of rows, such as BezierSurface::ControlPoints(), into a flat
// row-major vector.
template <typename Rows>
[[nodiscard]] auto Flatten(const Rows& rows)
{
    std::vector<std::ranges::range_value_t<std::ranges::range_value_t<Rows>>> flat;
    for (const auto& row : rows)
        flat.insert(flat.end(), row.begin(), row.end());
    return flat;
}

// Copies a flat row-major vector back into a range of mutable rows of the
// same shape, such as BezierSurface::ControlPoints().
template <typename Rows, typename T>
void Unflatten(std::span<const T> flat, Rows&& rows)
{
    std::size_t k = 0;
    for (auto row : rows)
    {
        for (auto& value : row)
            value = flat[k++];
    }
}

// The rows of a flat row-major vector with the given number of columns, as
// the surfaces' constructors take them.
template <typename T>
[[nodiscard]] std::vector<std::vector<T>> ToRows(std::span<const T> flat, std::size_t columns)
{
    std::vector<std::vector<T>> rows;
    for (std::size_t start = 0; start < flat.size(); start += columns)
        rows.emplace_back(flat.begin() + static_cast<std::ptrdiff_t>(start),
                          flat.begin() + static_cast<std::ptrdiff_t>(start + columns));
    return rows;
}

// The net for degree sliders set to newRows - 1 and newColumns - 1: the
// points kept, with each new column continuing its row's last leg and each
// new row its column's last leg (a fixed step along y or x when there is no
// leg yet), so a raised degree extends the net without folding it.
// Precondition: points has rows * columns elements, all counts >= 1.
[[nodiscard]] std::vector<glm::dvec3> ResizeControlNet(std::span<const glm::dvec3> points, std::size_t rows,
                                                       std::size_t columns, std::size_t newRows,
                                                       std::size_t newColumns);

// The weights for the same resize, with new weights 1.
[[nodiscard]] std::vector<double> ResizeWeights(std::span<const double> weights, std::size_t rows, std::size_t columns,
                                                std::size_t newRows, std::size_t newColumns);

} // namespace NURBS::Viewer
