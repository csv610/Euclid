#include <catch2/catch.hpp>
#include <Euclid/Render/Rasterizer.h>

#include <random>
#include <string>

#include <CGAL/Simple_cartesian.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/BoundingVolume/AABB.h>
#include <Euclid/Math/Vector.h>
#include <Eigen/Geometry>

#include <stb_image_write.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<float>;

TEST_CASE("Render, Rasterizer", "[render][rasterizer]")
{
    const uint32_t width = 100;
    const uint32_t height = 100;

    std::string filename(DATA_DIR);
    filename.append("bunny_vn.ply");
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<unsigned> indices;
    Euclid::read_ply<3>(
        filename, positions, &normals, nullptr, &indices, nullptr);

    Euclid::AABB<Kernel> aabb(positions);
    Eigen::Vector3f translation;
    Euclid::cgal_to_eigen(aabb.center(), translation);
    float scale = std::max(aabb.xlen(), aabb.ylen());
    scale = std::max(scale, aabb.zlen());
    Eigen::Transform<float, 3, Eigen::Affine> transform;
    transform.setIdentity();
    transform.translate(-translation);
    transform.scale(1.0f / scale);

    Eigen::Vector3f view(0.0f, 1.0f, 2.0f);
    Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Euclid::PerspRasCamera persp(
        view, center, up, 60.0f, width, height, 0.5f, 5.0f);

    const float xextent = 2.0f;
    const float yextent = xextent * height / (width + 1e-6f);
    Euclid::OrthoRasCamera ortho(
        view, center, up, xextent, yextent, 0.1f, 5.0f);

    Euclid::Rasterizer rasterizer(
        width, height, Euclid::Rasterizer::SAMPLE_COUNT_1);
    rasterizer.attach_position_buffer(positions.data(), positions.size());
    rasterizer.attach_normal_buffer(normals.data(), normals.size());
    rasterizer.attach_index_buffer(indices.data(), indices.size());

    Euclid::Light l;
    l.position = view + Eigen::Vector3f(0.1f, 0.1f, 0.1f);
    l.color = { 1.0f, 1.0f, 1.0f };
    l.intensity = 0.5f;
    rasterizer.set_light(l);

    SECTION("perspective camera")
    {
        std::vector<uint8_t> pixels(3 * width * height);
        rasterizer.render_shaded(transform.matrix(), persp, pixels);
        REQUIRE(pixels.size() == 3 * width * height);
    }

    SECTION("orthographic camera")
    {
        std::vector<uint8_t> pixels(3 * width * height);
        rasterizer.render_shaded(transform.matrix(), ortho, pixels);
        REQUIRE(pixels.size() == 3 * width * height);
    }

    SECTION("change material")
    {
        Euclid::Material material;
        material.ambient << 0.2f, 0.0f, 0.0f;
        material.diffuse << 0.7f, 0.0f, 0.0f;
        rasterizer.set_material(material);
        std::vector<uint8_t> pixels(3 * width * height);
        rasterizer.render_shaded(transform.matrix(), persp, pixels);
        REQUIRE(pixels.size() == 3 * width * height);
    }

    SECTION("change background")
    {
        rasterizer.set_background(0.0f, 0.3f, 0.4f);
        std::vector<uint8_t> pixels(3 * width * height);
        rasterizer.render_shaded(transform.matrix(), persp, pixels);
        REQUIRE(pixels.size() == 3 * width * height);
    }

    SECTION("random vertex color")
    {
        std::random_device rd;
        std::minstd_rand rd_gen(rd());
        std::uniform_real_distribution rd_number(0.0, 1.0);
        std::vector<float> rd_colors(positions.size());
        for (auto& color : rd_colors) {
            color = rd_number(rd_gen);
        }
        rasterizer.attach_color_buffer(rd_colors.data(), rd_colors.size());
        std::vector<uint8_t> pixels(3 * width * height);
        rasterizer.render_shaded(transform.matrix(), persp, pixels);
        REQUIRE(pixels.size() == 3 * width * height);
    }

    SECTION("unlit")
    {
        std::vector<uint8_t> pixels(3 * width * height);
        rasterizer.render_unlit(transform.matrix(), persp, pixels);
        REQUIRE(pixels.size() == 3 * width * height);
    }

    SECTION("depth")
    {
        std::vector<uint8_t> pixels(3 * width * height);
        rasterizer.render_depth(transform.matrix(), persp, pixels);
        REQUIRE(pixels.size() == 3 * width * height);

        std::vector<float> values(3 * width * height);
        rasterizer.render_depth(transform.matrix(), persp, values);
        REQUIRE(values.size() == 3 * width * height);
    }
}