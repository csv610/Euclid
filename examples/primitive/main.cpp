#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/PrimitiveGenerator.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

int main(int argc, char* argv[])
{
    const char* output = "sphere.ply";
    double radius = 1.0;
    int subdiv = 4;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-o" && i + 1 < argc) {
            output = argv[++i];
        } else if (std::string(argv[i]) == "--radius" && i + 1 < argc) {
            radius = std::atof(argv[++i]);
        } else if (std::string(argv[i]) == "--subdiv" && i + 1 < argc) {
            subdiv = std::atoi(argv[++i]);
        }
    }

    Mesh mesh;
    Euclid::make_subdivision_sphere(mesh, { 0.0, 0.0, 0.0 }, radius, subdiv);

    std::vector<double> positions;
    std::vector<unsigned> indices;
    Euclid::extract_mesh<3>(mesh, positions, nullptr, &indices);

    std::string fout(TMP_DIR);
    fout.append(output);
    Euclid::write_ply<3>(fout, positions, nullptr, nullptr, &indices, nullptr);
}
