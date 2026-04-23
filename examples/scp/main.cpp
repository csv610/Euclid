#include <fstream>
#include <string>
#include <unordered_map>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Parameterization/SCP.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Vertex = typename boost::graph_traits<Mesh>::vertex_descriptor;
using UV_Map = std::unordered_map<Vertex, Point_2>;
using UV_PMap = boost::associative_property_map<UV_Map>;

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
    } else if (ext == "stl") {
        Euclid::read_stl<3>(path, positions, normals, &indices, nullptr);
    }
}

int main(int argc, char* argv[])
{
    const char* input = "face.off";
    const char* output = "scp_texture.ply";

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-i" && i + 1 < argc) {
            input = argv[++i];
        } else if (std::string(argv[i]) == "-o" && i + 1 < argc) {
            output = argv[++i];
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

    auto bnd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;
    UV_Map uvm;
    UV_PMap uvpm(uvm);

    Euclid::SCP_parameterizer_3<Mesh> param;
    CGAL::Surface_mesh_parameterization::parameterize(mesh, param, bnd, uvpm);

    std::vector<double> texcoords(mesh.number_of_vertices() * 2);
    for (auto v : vertices(mesh)) {
        auto uv = uvm[v];
        texcoords[v.idx() * 2 + 0] = uv.x();
        texcoords[v.idx() * 2 + 1] = uv.y();
    }

    std::string fout(TMP_DIR);
    fout.append(output);
    Euclid::write_ply<3>(fout, positions, nullptr, &texcoords, &indices, nullptr);

    std::string fdisk(TMP_DIR);
    fdisk.append("scp_disk.off");
    std::ofstream ofs(fdisk);
    CGAL::Surface_mesh_parameterization::IO::output_uvmap_to_off(
        mesh, bnd, uvpm, ofs);
}