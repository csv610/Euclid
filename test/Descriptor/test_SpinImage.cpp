#include <catch2/catch.hpp>
#include <Euclid/Descriptor/SpinImage.h>

#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/PrimitiveGenerator.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/Descriptor/Histogram.h>
#include <Euclid/Util/Color.h>
#include <stb_image_write.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Vertex = Mesh::Vertex_index;

static void _write_spin_image(const std::string& f,
                              const Eigen::ArrayXd& si,
                              int width)
{
    auto vmax = si.maxCoeff();
    Eigen::ArrayXd sig = si;
    sig *= 255.0 / vmax;
    std::string fout(TMP_DIR);
    fout.append(f);
    stbi_write_png(
        fout.c_str(), width, width, 1, &sig(0), width * sizeof(double));
}

static void _write_distances_to_colored_mesh(
    const std::string& f,
    const std::vector<double>& positions,
    const std::vector<unsigned>& indices,
    const std::vector<double>& distances)
{
    std::vector<unsigned char> colors;
    Euclid::colormap(
        igl::COLOR_MAP_TYPE_JET, distances, colors, true, false, true);
    std::string fout(TMP_DIR);
    fout.append(f);
    Euclid::write_ply<3>(fout, positions, nullptr, nullptr, &indices, &colors);
}

TEST_CASE("Descriptor, SpinImage", "[descriptor][spinimage]")
{
    Mesh mesh;
    Euclid::make_subdivision_sphere(mesh, Point_3(0, 0, 0), 1.0, 2);

    std::vector<double> positions;
    std::vector<unsigned> indices;
    Euclid::extract_mesh<3>(mesh, positions, indices);

    auto idx1 = 0;
    auto idx2 = mesh.vertices().size() / 2;
    auto idx3 = mesh.vertices().size() / 3;
    auto idx4 = mesh.vertices().size() - 1;

    SECTION("default support")
    {
        const unsigned width = 16;
        const float angle = 90.0f;

        Euclid::SpinImage<Mesh> si;
        si.build(mesh);

        Eigen::ArrayXXd si_all;
        si.compute(si_all, 1.0f, width, angle);

        std::vector<double> distances(si_all.cols());
        for (int i = 0; i < si_all.cols(); ++i) {
            distances[i] = Euclid::chi2(si_all.col(i), si_all.col(idx1));
        }
        REQUIRE(distances[idx1] == 0);
        REQUIRE(distances[idx2] < distances[idx4]);
        REQUIRE(distances[idx3] < distances[idx4]);

        _write_spin_image("spinimage1_1.png", si_all.col(idx1), width);
        _write_spin_image("spinimage1_2.png", si_all.col(idx2), width);
        _write_spin_image("spinimage1_3.png", si_all.col(idx3), width);
        _write_spin_image("spinimage1_4.png", si_all.col(idx4), width);

        _write_distances_to_colored_mesh(
            "spinimage1.ply", positions, indices, distances);
    }

    SECTION("smaller support")
    {
        const unsigned width = 8;
        const float angle = 60.0f;

        Euclid::SpinImage<Mesh> si;
        si.build(mesh);

        Eigen::ArrayXXd si_all;
        si.compute(si_all, 1.0f, width, angle);

        std::vector<double> distances(si_all.cols());
        for (int i = 0; i < si_all.cols(); ++i) {
            distances[i] = Euclid::chi2(si_all.col(i), si_all.col(idx1));
        }
        REQUIRE(distances[idx1] == 0);
        REQUIRE(distances[idx2] < distances[idx4]);
        REQUIRE(distances[idx3] < distances[idx4]);

        _write_spin_image("spinimage2_1.png", si_all.col(idx1), width);
        _write_spin_image("spinimage2_2.png", si_all.col(idx2), width);
        _write_spin_image("spinimage2_3.png", si_all.col(idx3), width);
        _write_spin_image("spinimage2_4.png", si_all.col(idx4), width);

        _write_distances_to_colored_mesh(
            "spinimage2.ply", positions, indices, distances);
    }

    SECTION("larger support")
    {
        const unsigned width = 32;
        const float angle = 120.0f;

        Euclid::SpinImage<Mesh> si;
        si.build(mesh);

        Eigen::ArrayXXd si_all;
        si.compute(si_all, 1.0f, width, angle);

        std::vector<double> distances(si_all.cols());
        for (int i = 0; i < si_all.cols(); ++i) {
            distances[i] = Euclid::chi2(si_all.col(i), si_all.col(idx1));
        }
        REQUIRE(distances[idx1] == 0);
        REQUIRE(distances[idx2] < distances[idx4]);
        REQUIRE(distances[idx3] < distances[idx4]);

        _write_spin_image("spinimage3_1.png", si_all.col(idx1), width);
        _write_spin_image("spinimage3_2.png", si_all.col(idx2), width);
        _write_spin_image("spinimage3_3.png", si_all.col(idx3), width);
        _write_spin_image("spinimage3_4.png", si_all.col(idx4), width);

        _write_distances_to_colored_mesh(
            "spinimage3.ply", positions, indices, distances);
    }
}
