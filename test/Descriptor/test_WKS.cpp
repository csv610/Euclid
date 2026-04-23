#include <catch2/catch.hpp>
#include <Euclid/Descriptor/WKS.h>

#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/PrimitiveGenerator.h>
#include <Euclid/Geometry/Spectral.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/Descriptor/Histogram.h>
#include <Euclid/Util/Color.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

TEST_CASE("Descriptor, WKS", "[descriptor][wks]")
{
    Mesh mesh;
    Euclid::make_subdivision_sphere(mesh, Point_3(0, 0, 0), 1.0, 2);

    std::vector<double> positions;
    std::vector<unsigned> indices;
    Euclid::extract_mesh<3>(mesh, positions, indices);

    auto n = static_cast<int>(mesh.vertices().size());

    constexpr const int ne = 10;
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenfunctions;
    Euclid::spectrum(mesh, ne, eigenvalues, eigenfunctions);

    REQUIRE(eigenvalues(1) > eigenvalues(0));
    REQUIRE(eigenvalues(2) > eigenvalues(1));
    REQUIRE(eigenfunctions.rows() == n);

    SECTION("build with mesh and k")
    {
        Euclid::WKS<Mesh> wks1;
        wks1.build(mesh, ne);
        Eigen::ArrayXXd result;
        wks1.compute(result, 10, 0.1f, 10.0f, 0.5f);
        REQUIRE(result.cols() == n);
    }

    SECTION("build with eigenvalues")
    {
        Euclid::WKS<Mesh> wks;
        wks.build(mesh, &eigenvalues, &eigenfunctions);
        Eigen::ArrayXXd result;
        wks.compute(result, 10, 0.1f, 10.0f, 0.5f);
        REQUIRE(result.cols() == n);
        REQUIRE(result.rows() == 10);
    }

    SECTION("compute with explicit params")
    {
        Euclid::WKS<Mesh> wks;
        wks.build(mesh, &eigenvalues, &eigenfunctions);

        constexpr int n_scales = 50;
        constexpr double emin = 0.1;
        constexpr double emax = 10.0;
        constexpr double sigma = 0.5;

        Eigen::ArrayXXd wks_all;
        wks.compute(wks_all, n_scales, emin, emax, sigma);

        REQUIRE(wks_all.cols() == n);
        REQUIRE(wks_all.rows() == n_scales);
        REQUIRE(wks_all(0, 0) > 0);
    }

    SECTION("chi2 distance")
    {
        Euclid::WKS<Mesh> wks;
        wks.build(mesh, &eigenvalues, &eigenfunctions);

        Eigen::ArrayXXd wks_all;
        wks.compute(wks_all, 50, 0.1, 10.0, 0.5);

        auto d = Euclid::chi2(wks_all.col(0), wks_all.col(0));
        REQUIRE(d == 0);

        for (int i = 1; i < n; ++i) {
            REQUIRE(Euclid::chi2(wks_all.col(i), wks_all.col(0)) > 0);
        }
    }

    SECTION("different energy ranges produce different results")
    {
        Euclid::WKS<Mesh> wks;
        wks.build(mesh, &eigenvalues, &eigenfunctions);

        Eigen::ArrayXXd wks1, wks2;
        wks.compute(wks1, 20, 0.1, 5.0, 0.3);
        wks.compute(wks2, 20, 5.0, 20.0, 0.3);

        REQUIRE(wks1.rows() == 20);
        REQUIRE(wks2.rows() == 20);
        REQUIRE(wks1(0, 0) != wks2(0, 0));
    }
}