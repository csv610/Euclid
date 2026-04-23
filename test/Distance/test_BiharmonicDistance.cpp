#include <catch2/catch.hpp>
#include <Euclid/Distance/BiharmonicDistance.h>

#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/Geometry/Spectral.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/PrimitiveGenerator.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

TEST_CASE("Distance, Biharmonic distance", "[distance][biharmonicdistance]")
{
    Mesh mesh;
    Euclid::make_subdivision_sphere(mesh, Point_3(0, 0, 0), 1.0, 2);

    auto n = static_cast<int>(mesh.vertices().size());
    constexpr int k = 10;

    Eigen::VectorXd lambdas;
    Eigen::MatrixXd phis;
    Euclid::spectrum(mesh, k, lambdas, phis);

    SECTION("distance to self is zero")
    {
        auto d = Euclid::biharmonic_distance(lambdas, phis, 0, 0);
        REQUIRE(d == Approx(0.0));
    }

    SECTION("distance positive for different vertices")
    {
        auto d = Euclid::biharmonic_distance(lambdas, phis, 0, 5);
        REQUIRE(d > 0);
    }

    SECTION("symmetric distance")
    {
        auto d1 = Euclid::biharmonic_distance(lambdas, phis, 0, 5);
        auto d2 = Euclid::biharmonic_distance(lambdas, phis, 5, 0);
        REQUIRE(d1 == Approx(d2));
    }

    SECTION("triangle inequality (relaxed)")
    {
        auto d01 = Euclid::biharmonic_distance(lambdas, phis, 0, 1);
        auto d12 = Euclid::biharmonic_distance(lambdas, phis, 1, 2);
        auto d02 = Euclid::biharmonic_distance(lambdas, phis, 0, 2);
        REQUIRE(d02 <= d01 + d12 + 1e-6);
    }

    SECTION("compute all pairwise distances")
    {
        std::vector<double> distances(n);
        for (int i = 0; i < n; ++i) {
            distances[i] = Euclid::biharmonic_distance(lambdas, phis, 0, i);
        }
        REQUIRE(distances.size() == static_cast<size_t>(n));
        REQUIRE(distances[0] == Approx(0.0));
        for (int i = 1; i < n; ++i) {
            REQUIRE(distances[i] > 0);
        }
    }
}