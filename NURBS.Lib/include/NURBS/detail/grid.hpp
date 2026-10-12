/// @file
/// Detail::Grid, a rectangular grid of values stored flat in row-major order.

#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <ranges>
#include <span>
#include <vector>

namespace NURBS
{

namespace Detail
{

// Views a flat row-major array as a random-access range of rows, each a span of `columns` elements.
template <typename T>
[[nodiscard]] constexpr auto Rows(std::span<T> flat, std::size_t columns)
{
    return std::views::iota(std::size_t{0}, flat.size() / columns) |
           std::views::transform([flat, columns](std::size_t i) { return flat.subspan(i * columns, columns); });
}

// A rectangular grid of values stored flat in row-major order, so rows cannot differ in length.
template <typename T>
class Grid
{
public:
    // Copies a range of rows. Pre: there is at least one row, the rows are non-empty, and all have the same size.
    template <std::ranges::input_range R>
        requires std::ranges::input_range<std::ranges::range_reference_t<R>> &&
                 std::convertible_to<std::ranges::range_reference_t<std::ranges::range_reference_t<R>>, T>
    explicit constexpr Grid(const R& rows)
    {
        std::size_t rowCount = 0;
        for (const auto& row : rows)
        {
            const std::size_t before = m_values.size();
            for (const auto& value : row)
                m_values.push_back(static_cast<T>(value));

            const std::size_t length = m_values.size() - before;
            if (rowCount == 0)
                m_columns = length;
            assert(length == m_columns);
            ++rowCount;
        }
        assert(rowCount > 0);
        assert(m_columns > 0);
    }

    // A rows-by-columns grid with every element equal to value.
    constexpr Grid(std::size_t rows, std::size_t columns, const T& value)
        : m_values(rows * columns, value), m_columns(columns)
    {
        assert(rows > 0);
        assert(columns > 0);
    }

    [[nodiscard]] constexpr std::size_t RowCount() const { return m_values.size() / m_columns; }
    [[nodiscard]] constexpr std::size_t ColumnCount() const { return m_columns; }

    [[nodiscard]] constexpr std::span<const T> Values() const { return m_values; }

    [[nodiscard]] constexpr auto Rows() const { return Detail::Rows(std::span<const T>{m_values}, m_columns); }
    [[nodiscard]] constexpr auto Rows() { return Detail::Rows(std::span<T>{m_values}, m_columns); }

private:
    std::vector<T> m_values;
    std::size_t m_columns = 0;
};

} // namespace Detail

} // namespace NURBS
