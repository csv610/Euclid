#include <catch2/catch.hpp>
#include <Euclid/Descriptor/HKS.h>

#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/PrimitiveGenerator.h>
#include <Euclid/Geometry/Spectral.h>
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

TEST_CASE("Descriptor,HKS", "[descriptor][hks]")
{
    Mesh mesh;
    Euclid::make_subdivision_sphere(mesh, Point_3(0, 0, 0), 1.0, 1);

    std::vector<double> positions;
    std::vector<unsigned> indices;
    Euclid::extract_mesh<3>(mesh, positions, indices);

    auto n = static_cast<int>(mesh.vertices().size());
    auto idx1 = 0;

    constexpr const int ne = 10;
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenfunctions;
    Euclid::spectrum(mesh, ne, eigenvalues, eigenfunctions);

    Euclid::HKS<Mesh> hks;
    hks.build(mesh, &eigenvalues, &eigenfunctions);

    SECTION("default time range")
    {
        Eigen::ArrayXXd hks_all;
        hks.compute(hks_all);

        REQUIRE(hks_all.cols() == n);
        REQUIRE(hks_all.rows() > 0);
        REQUIRE(hks_all(0, idx1) > 0);

        std::vector<double> distances(n);
        for (int i = 0; i < n; ++i) {
            distances[i] = Euclid::chi2(hks_all.col(i), hks_all.col(idx1));
        }
        REQUIRE(distances[idx1] == 0);

        _write_distances_to_colored_mesh(
            "hks1.ply", positions, indices, distances);
    }

    SECTION("smaller time range")
    {
        auto c = std::log(10.0);
        auto tmin = c / eigenvalues(eigenvalues.size() - 1);
        auto tmax = c / eigenvalues(1);
        Eigen::ArrayXXd hks_all;
        hks.compute(hks_all, 50, tmin, tmax);

        REQUIRE(hks_all.cols() == n);
        REQUIRE(hks_all.rows() == 50);

        _write_distances_to_colored_mesh(
            "hks2.ply", positions, indices, std::vector<double>(n, 0));
    }
}