#include "ControlNet.hpp"

#include <algorithm>
#include <cassert>

namespace NURBS::Viewer
{

namespace
{

// The step a new row (along u, so x) or column (along v, so y) takes when the
// net has only one row or column to continue.
constexpr glm::dvec3 kRowStep{0.5, 0.0, 0.0};
constexpr glm::dvec3 kColumnStep{0.0, 0.5, 0.0};

// Copies the kept corner of a rows x columns grid into a newRows x newColumns
// grid filled with fill.
template <typename T>
[[nodiscard]] std::vector<T> KeepCorner(std::span<const T> values, std::size_t rows, std::size_t columns,
                                        std::size_t newRows, std::size_t newColumns, const T& fill)
{
    assert(values.size() == rows * columns);
    assert(newRows >= 1 && newColumns >= 1);

    std::vector<T> resized(newRows * newColumns, fill);
    for (std::size_t i = 0; i < std::min(rows, newRows); ++i)
    {
        for (std::size_t j = 0; j < std::min(columns, newColumns); ++j)
            resized[i * newColumns + j] = values[i * columns + j];
    }
    return resized;
}

} // namespace

std::vector<glm::dvec3> ResizeControlNet(std::span<const glm::dvec3> points, std::size_t rows, std::size_t columns,
                                         std::size_t newRows, std::size_t newColumns)
{
    std::vector<glm::dvec3> net = KeepCorner(points, rows, columns, newRows, newColumns, glm::dvec3{0.0});
    const auto at = [&net, newColumns](std::size_t i, std::size_t j) -> glm::dvec3& { return net[i * newColumns + j]; };

    // New columns of the kept rows, then whole new rows.
    for (std::size_t i = 0; i < std::min(rows, newRows); ++i)
    {
        for (std::size_t j = columns; j < newColumns; ++j)
            at(i, j) = at(i, j - 1) + (j >= 2 ? at(i, j - 1) - at(i, j - 2) : kColumnStep);
    }
    for (std::size_t i = rows; i < newRows; ++i)
    {
        for (std::size_t j = 0; j < newColumns; ++j)
            at(i, j) = at(i - 1, j) + (i >= 2 ? at(i - 1, j) - at(i - 2, j) : kRowStep);
    }
    return net;
}

std::vector<double> ResizeWeights(std::span<const double> weights, std::size_t rows, std::size_t columns,
                                  std::size_t newRows, std::size_t newColumns)
{
    return KeepCorner(weights, rows, columns, newRows, newColumns, 1.0);
}

} // namespace NURBS::Viewer
