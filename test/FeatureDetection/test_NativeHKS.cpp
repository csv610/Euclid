#include <catch2/catch.hpp>
#include <Euclid/FeatureDetection/NativeHKS.h>

#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/Descriptor/HKS.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/PrimitiveGenerator.h>
#include <Euclid/Math/Numeric.h>
#include <Euclid/Geometry/Spectral.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/Util/Color.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Vertex = Mesh::Vertex_index;

static void _write_features_to_colored_mesh(
    const std::string& f,
    const std::vector<double>& positions,
    const std::vector<unsigned>& indices,
    const std::vector<Vertex>& features)
{
    std::vector<unsigned char> colors(positions.size(), 0);
    for (auto v : features) {
        colors[v.idx() * 3] = 255;
    }
    std::string fout(TMP_DIR);
    fout.append(f);
    Euclid::write_ply<3>(fout, positions, nullptr, nullptr, &indices, &colors);
}

TEST_CASE("Feature detection, Native HKS", "[feature][nativehks]")
{
    Mesh mesh;
    Euclid::make_subdivision_sphere(mesh, Point_3(0, 0, 0), 1.0, 2);

    std::vector<double> positions;
    std::vector<unsigned> indices;
    Euclid::extract_mesh<3>(mesh, positions, indices);

    constexpr const int ne = 30;
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenfunctions;
    Euclid::spectrum(mesh, ne, eigenvalues, eigenfunctions);

    Euclid::HKS<Mesh> hks;
    hks.build(mesh, &eigenvalues, &eigenfunctions);

    SECTION("smaller time range")
    {
        Eigen::ArrayXXd hks_all;
        hks.compute(hks_all, 100, 0.1, 4.0);
        auto features =
            Euclid::native_hks_features(mesh, hks_all, hks_all.rows() - 1);
        _write_features_to_colored_mesh("native_hks_features_smaller.ply",
                                        positions,
                                        indices,
                                        features);
    }

    SECTION("larger time range")
    {
        Eigen::ArrayXXd hks_all;
        hks.compute(hks_all, 100, 0.1, 8.0);
        auto features =
            Euclid::native_hks_features(mesh, hks_all, hks_all.rows() - 1);
        _write_features_to_colored_mesh("native_hks_features_larger.ply",
                                        positions,
                                        indices,
                                        features);
    }
}