#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/Descriptor/WKS.h>
#include <Euclid/Geometry/Spectral.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Util/Color.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;

int main()
{
    std::vector<double> positions;
    std::vector<double> normals;
    std::vector<unsigned> indices;
    std::string fmesh(DATA_DIR);
    fmesh.append("dragon.ply");
    Euclid::read_ply<3>(fmesh, positions, &normals, nullptr, &indices, nullptr);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);

    constexpr int ne = 300;
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenfunctions;
    Euclid::spectrum(mesh, ne, eigenvalues, eigenfunctions);

    Euclid::WKS<Mesh> wks;
    wks.build(mesh, &eigenvalues, &eigenfunctions);

    Eigen::ArrayXXd wks_all;
    wks.compute(wks_all, 100);

    auto idx = 21785;
    std::vector<double> distances(wks_all.cols());
    for (int i = 0; i < wks_all.cols(); ++i) {
        distances[i] = Euclid::chi2(wks_all.col(i), wks_all.col(idx));
    }

    std::vector<unsigned char> colors;
    Euclid::colormap(igl::COLOR_MAP_TYPE_JET, distances, colors, true, false, true);

    std::string fout(TMP_DIR);
    fout.append("wks_distances.ply");
    Euclid::write_ply<3>(fout, positions, nullptr, nullptr, &indices, &colors);
}