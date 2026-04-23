#include <iostream>
#include <string>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Topology/MeshTopology.h>
#include <Euclid/Topology/HomologyGenerator.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

static std::string get_extension(const std::string& path)
{
    auto pos = path.rfind('.');
    if (pos != std::string::npos) {
        return path.substr(pos + 1);
    }
    return "";
}

template<typename T>
void read_mesh(const std::string& path,
              std::vector<T>& positions,
              std::vector<T>* normals,
              std::vector<unsigned>& indices)
{
    std::string ext = get_extension(path);
    if (ext == "off") {
        Euclid::read_off<3>(path, positions, normals, &indices, nullptr);
    } else if (ext == "ply") {
        Euclid::read_ply<3>(path, positions, normals, nullptr, &indices, nullptr);
    } else if (ext == "obj") {
        Euclid::read_obj<3>(path, positions, normals, &indices, nullptr);
    }
}

int main(int argc, char* argv[])
{
    const char* input = "kitten.off";
    double accept = 0.05;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-i" && i + 1 < argc) {
            input = argv[++i];
        } else if (std::string(argv[i]) == "--accept" && i + 1 < argc) {
            accept = std::atof(argv[++i]);
        }
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;
    std::string fmesh(DATA_DIR);
    fmesh.append(input);
    read_mesh(fmesh, positions, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);

    int boundaries = Euclid::num_boundaries(mesh);
    int euler = Euclid::euler_characteristic(mesh);
    int g = Euclid::genus(mesh);

    std::cout << "Boundaries: " << boundaries << std::endl;
    std::cout << "Euler characteristic: " << euler << std::endl;
    std::cout << "Genus: " << g << std::endl;

    auto generators = Euclid::greedy_homology_generators(mesh, accept);
    std::cout << "Homology generators: " << generators.size() << std::endl;
    for (size_t i = 0; i < generators.size(); ++i) {
        std::cout << "Generator " << i << ": " << generators[i].size() << " vertices" << std::endl;
    }
}
