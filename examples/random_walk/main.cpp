#include <unordered_map>
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Segmentation/RandomWalk.h>
#include <Euclid/Util/Color.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<float>;
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
    const char* input = "bunny_vn.ply";
    const char* output = "random_walk_segment.ply";

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-i" && i + 1 < argc) {
            input = argv[++i];
        } else if (std::string(argv[i]) == "-o" && i + 1 < argc) {
            output = argv[++i];
        }
    }

    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<unsigned> indices;
    std::string fmesh(DATA_DIR);
    fmesh.append(input);
    read_mesh(fmesh, positions, &normals, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);

    std::vector<unsigned> seeds{ 11586, 5052,  248,  3995,  4709,  10824,
                                 7158,  14057, 1630, 13981, 14179, 4532 };
    std::vector<unsigned> segments;
    Euclid::random_walk_segmentation(mesh, seeds, segments);

    std::unordered_map<unsigned, unsigned> sid;
    for (size_t i = 0; i < seeds.size(); ++i) {
        sid.emplace(seeds[i], i);
    }
    std::vector<unsigned char> color_set;
    Euclid::rnd_colors(seeds.size(), color_set, true);
    std::vector<unsigned char> colors(segments.size() * 4);
    for (size_t i = 0; i < segments.size(); ++i) {
        auto id = sid[segments[i]];
        colors[4 * i + 0] = color_set[3 * id + 0];
        colors[4 * i + 1] = color_set[3 * id + 1];
        colors[4 * i + 2] = color_set[3 * id + 2];
        colors[4 * i + 3] = 255;
    }
    std::string fout(TMP_DIR);
    fout.append(output);
    Euclid::write_ply<3>(fout, positions, nullptr, nullptr, &indices, &colors);
}