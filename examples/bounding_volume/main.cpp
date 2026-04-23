#include <iostream>
#include <string>
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/BoundingVolume/AABB.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;

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

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-i" && i + 1 < argc) {
            input = argv[++i];
        }
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;
    std::string fmesh(DATA_DIR);
    fmesh.append(input);
    read_mesh(fmesh, positions, nullptr, indices);

    Euclid::AABB<Kernel> aabb(positions);

    std::cout << "AABB center: ("
              << aabb.center().x() << ", "
              << aabb.center().y() << ", "
              << aabb.center().z() << ")" << std::endl;
    std::cout << "X: [" << aabb.xmin() << ", " << aabb.xmax() << "]" << std::endl;
    std::cout << "Y: [" << aabb.ymin() << ", " << aabb.ymax() << "]" << std::endl;
    std::cout << "Z: [" << aabb.zmin() << ", " << aabb.zmax() << "]" << std::endl;
    std::cout << "Dimensions: "
             << aabb.xlen() << " x "
             << aabb.ylen() << " x "
             << aabb.zlen() << std::endl;
}
