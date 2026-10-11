#include <gtest/gtest.h>

#include <Graphics/SurfaceMesh.hpp>

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace
{

// The plane z = 0 with u along x and v along y, so S_u x S_v = +z.
glm::dvec3 Plane(double u, double v)
{
    return {u, v, 0.0};
}

} // namespace

TEST(SampleGrid, RowMajor)
{
    const std::vector<glm::dvec3> points = Graphics::SampleGrid(Plane, 0.0, 2.0, 10.0, 13.0, 3, 4);
    ASSERT_EQ(points.size(), 12u);
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            EXPECT_EQ(points[i * 4 + j], glm::dvec3(static_cast<double>(i), 10.0 + static_cast<double>(j), 0.0));
        }
    }
}

TEST(SampleGrid, ConvertsToDouble)
{
    const auto single = [](double u, double v) { return glm::vec3(u, v, 0.5); };
    const std::vector<glm::dvec3> points = Graphics::SampleGrid(single, 0.0, 1.0, 0.0, 1.0, 2, 2);
    EXPECT_EQ(points.back(), glm::dvec3(1.0, 1.0, 0.5));
}

TEST(GridNormals, Plane)
{
    const std::vector<glm::dvec3> points = Graphics::SampleGrid(Plane, 0.0, 1.0, 0.0, 1.0, 4, 3);
    const std::vector<glm::dvec3> normals = Graphics::GridNormals(points, 4, 3);
    ASSERT_EQ(normals.size(), points.size());
    for (const glm::dvec3& n : normals)
    {
        EXPECT_EQ(n, glm::dvec3(0.0, 0.0, 1.0));
    }
}

TEST(GridNormals, SphereIsUnitAndRadial)
{
    // A patch of the unit sphere away from the poles, with u the longitude and v the latitude, so S_u x S_v points
    // outward.
    const auto sphere = [](double u, double v) {
        return glm::dvec3(std::cos(v) * std::cos(u), std::cos(v) * std::sin(u), std::sin(v));
    };
    constexpr std::size_t kCount = 9;
    const std::vector<glm::dvec3> points = Graphics::SampleGrid(sphere, 0.0, 1.0, -0.5, 0.5, kCount, kCount);
    const std::vector<glm::dvec3> normals = Graphics::GridNormals(points, kCount, kCount);
    for (std::size_t k = 0; k < points.size(); ++k)
    {
        EXPECT_NEAR(glm::length(normals[k]), 1.0, 1e-12);
        // Central differences track the radial direction closely; one-sided ones on the edges less so.
        EXPECT_GT(glm::dot(normals[k], points[k]), 0.99);
    }
}

TEST(GridNormals, DegenerateFallsBackToZ)
{
    const std::vector<glm::dvec3> points(6, glm::dvec3(1.0, 2.0, 3.0));
    const std::vector<glm::dvec3> normals = Graphics::GridNormals(points, 2, 3);
    for (const glm::dvec3& n : normals)
    {
        EXPECT_EQ(n, glm::dvec3(0.0, 0.0, 1.0));
    }
}

TEST(GridTriangleIndices, SingleCell)
{
    // With vCount = 2: (0,0) = 0, (0,1) = 1, (1,0) = 2, (1,1) = 3.
    const std::vector<std::uint32_t> indices = Graphics::GridTriangleIndices(1, 1);
    EXPECT_EQ(indices, (std::vector<std::uint32_t>{0, 2, 3, 0, 3, 1}));
}

TEST(GridTriangleIndices, CountAndRange)
{
    const std::vector<std::uint32_t> indices = Graphics::GridTriangleIndices(3, 2);
    EXPECT_EQ(indices.size(), 6u * 3u * 2u);
    for (const std::uint32_t index : indices)
    {
        EXPECT_LT(index, 4u * 3u);
    }
}

TEST(GridTriangleIndices, CounterclockwiseAboutNormal)
{
    constexpr std::size_t kUSegments = 3;
    constexpr std::size_t kVSegments = 2;
    const std::vector<glm::dvec3> points =
        Graphics::SampleGrid(Plane, 0.0, 1.0, 0.0, 1.0, kUSegments + 1, kVSegments + 1);
    const std::vector<std::uint32_t> indices = Graphics::GridTriangleIndices(kUSegments, kVSegments);
    for (std::size_t t = 0; t < indices.size(); t += 3)
    {
        const glm::dvec3 a = points[indices[t]];
        const glm::dvec3 b = points[indices[t + 1]];
        const glm::dvec3 c = points[indices[t + 2]];
        EXPECT_GT(glm::cross(b - a, c - a).z, 0.0);
    }
}

TEST(MeshVertices, ConvertsToFloat)
{
    const std::vector<glm::dvec3> points{{1.0, 2.0, 3.0}, {0.5, 0.25, 0.125}};
    const std::vector<glm::dvec3> normals{{0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}};
    const std::vector<Graphics::MeshVertex> vertices = Graphics::MeshVertices(points, normals);
    ASSERT_EQ(vertices.size(), 2u);
    EXPECT_EQ(vertices[0].position, glm::vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(vertices[0].normal, glm::vec3(0.0f, 0.0f, 1.0f));
    EXPECT_EQ(vertices[1].position, glm::vec3(0.5f, 0.25f, 0.125f));
    EXPECT_EQ(vertices[1].normal, glm::vec3(1.0f, 0.0f, 0.0f));
}

TEST(SampleSurfaceMesh, Sizes)
{
    const Graphics::MeshData mesh = Graphics::SampleSurfaceMesh(Plane, 0.0, 1.0, 0.0, 1.0, 4, 3);
    EXPECT_EQ(mesh.vertices.size(), 5u * 4u);
    EXPECT_EQ(mesh.indices.size(), 6u * 4u * 3u);
    EXPECT_EQ(mesh.vertices.back().position, glm::vec3(1.0f, 1.0f, 0.0f));
    EXPECT_EQ(mesh.vertices.back().normal, glm::vec3(0.0f, 0.0f, 1.0f));
}
