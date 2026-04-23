#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Eigen/Core>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/BoundingVolume/AABB.h>
#include <Euclid/BoundingVolume/OBB.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Mat = Eigen::MatrixXd;

std::string get_extension(const std::string& path)
{
    auto pos = path.rfind('.');
    if (pos != std::string::npos) {
        return path.substr(pos + 1);
    }
    return "";
}

void print_usage(const char* program)
{
    std::cout << "Usage: " << program << " [options]\n"
              << "Bounding volume computation tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file (with bounding box)\n"
              << "  --aabb       Compute Axis-Aligned Bounding Box\n"
              << "  --obb        Compute Oriented Bounding Box\n"
              << "  --both       Compute both AABB and OBB\n"
              << "  --help       Show this help message\n";
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
    std::string input_file;
    std::string output_file;
    bool do_aabb = false;
    bool do_obb = false;
    bool do_both = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--aabb") == 0) {
            do_aabb = true;
        } else if (strcmp(argv[i], "--obb") == 0) {
            do_obb = true;
        } else if (strcmp(argv[i], "--both") == 0) {
            do_both = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (do_both) {
        do_aabb = do_obb = true;
    }

    if (!do_aabb && !do_obb) {
        do_aabb = true;
    }

    if (input_file.empty()) {
        std::cerr << "Error: No input file specified\n\n";
        print_usage(argv[0]);
        return 1;
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;

    std::cout << "Reading mesh from " << input_file << "...\n";
    read_mesh(input_file, positions, nullptr, indices);

    auto nv = positions.size() / 3;
    std::cout << "Loaded " << nv << " vertices, "
              << (indices.size() / 3) << " faces.\n";

    if (do_aabb) {
        std::cout << "Computing AABB...\n";
        Euclid::AABB<Kernel> aabb(positions);

        std::cout << "  Center: (" << aabb.center() << ")\n";
        std::cout << "  X: [" << aabb.xmin() << ", " << aabb.xmax() << "] len=" << aabb.xlen() << "\n";
        std::cout << "  Y: [" << aabb.ymin() << ", " << aabb.ymax() << "] len=" << aabb.ylen() << "\n";
        std::cout << "  Z: [" << aabb.zmin() << ", " << aabb.zmax() << "] len=" << aabb.zlen() << "\n";

        if (!output_file.empty()) {
            std::vector<double> box_positions;
            std::vector<unsigned> box_indices;
            Euclid::generate_box(box_positions, box_indices,
                             aabb.xmin(), aabb.xmax(),
                             aabb.ymin(), aabb.ymax(),
                             aabb.zmin(), aabb.zmax());

            std::stringstream ss;
            ss << output_file;
            auto pos = ss.str().rfind('.');
            if (pos != std::string::npos) {
                ss.str(ss.str().insert(pos, "_aabb"));
            }
            std::cout << "Writing AABB to " << ss.str() << "...\n";
            Euclid::write_off<3>(ss.str(), box_positions, nullptr, &box_indices, nullptr);
        }
    }

    if (do_obb) {
        std::cout << "Computing OBB...\n";
        Euclid::OBB<Kernel> obb;
        obb.build(positions);

        std::cout << "  Center: (" << obb.center() << ")\n";
        for (int i = 0; i < 3; ++i) {
            std::cout << "  Axis " << i << ": (" << obb.axis(i) << ") len=" << obb.length(i) << "\n";
        }

        if (!output_file.empty()) {
            std::stringstream ss;
            ss << output_file;
            auto pos = ss.str().rfind('.');
            if (pos != std::string::npos) {
                ss.str(ss.str().insert(pos, "_obb"));
            }
            std::cout << "Writing OBB to " << ss.str() << "...\n";
        }
    }

    std::cout << "Done.\n";
    return 0;
}