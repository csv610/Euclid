#include <iostream>
#include <string>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/ViewSelection/ViewSphere.h>

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
    const char* input = "bunny.off";
    const char* output = "view_sphere.off";
    float scale = 3.0f;
    int subdiv = 4;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-i" && i + 1 < argc) {
            input = argv[++i];
        } else if (std::string(argv[i]) == "-o" && i + 1 < argc) {
            output = argv[++i];
        } else if (std::string(argv[i]) == "--scale" && i + 1 < argc) {
            scale = std::atof(argv[++i]);
        } else if (std::string(argv[i]) == "--subdiv" && i + 1 < argc) {
            subdiv = std::atoi(argv[++i]);
        }
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;
    std::string fmesh(DATA_DIR);
    fmesh.append(input);
    read_mesh(fmesh, positions, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);

    auto sphere = Euclid::ViewSphere<Mesh>::make_subdiv(mesh, scale, subdiv);

    std::cout << "View sphere center: ("
              << sphere.center.x() << ", "
              << sphere.center.y() << ", "
              << sphere.center.z() << ")" << std::endl;
    std::cout << "View sphere radius: " << sphere.radius << std::endl;

    std::vector<double> spositions;
    std::vector<unsigned> sindices;
    Euclid::extract_mesh<3>(sphere.mesh, spositions, &sindices);

    std::string fout(TMP_DIR);
    fout.append(output);
    Euclid::write_off<3>(fout, spositions, nullptr, &sindices, nullptr);
}