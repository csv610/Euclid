#include <vector>
#include <string>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/Distance/GeodesicsInHeat.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Util/Color.h>

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
    const char* output = "geodesics.ply";
    int source = 0;
    float scale = 4.0f;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-i" && i + 1 < argc) {
            input = argv[++i];
        } else if (std::string(argv[i]) == "-o" && i + 1 < argc) {
            output = argv[++i];
        } else if (std::string(argv[i]) == "-s" && i + 1 < argc) {
            source = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "--scale" && i + 1 < argc) {
            scale = std::atof(argv[++i]);
        }
    }

    std::vector<double> positions;
    std::vector<double> normals;
    std::vector<unsigned> indices;
    std::string fmesh(DATA_DIR);
    fmesh.append(input);
    read_mesh(fmesh, positions, &normals, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);

    Euclid::GeodesicsInHeat<Mesh> heat_method;
    heat_method.build(mesh, scale);

    std::vector<double> geodesics;
    heat_method.compute(Mesh::Vertex_index(source), geodesics);

    std::vector<unsigned char> colors;
    Euclid::colormap(igl::COLOR_MAP_TYPE_PARULA, geodesics, colors, true, true);

    std::string fout(TMP_DIR);
    fout.append(output);
    Euclid::write_ply<3>(fout, positions, nullptr, nullptr, &indices, &colors);
}