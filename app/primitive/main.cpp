#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/PrimitiveGenerator.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

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
              << "Generate mesh primitives\n\n"
              << "Options:\n"
              << "  -o <file>    Output mesh file\n"
              << "  --sphere    Generate sphere\n"
              << "  --cube     Generate cube\n"
              << "  --cylinder Generate cylinder\n"
              << "  --cone    Generate cone\n"
              << "  --torus   Generate torus\n"
              << "  --plane   Generate plane\n"
              << "  --ico    Generate icosphere\n"
              << "  -r <num>   Radius (for sphere, torus, cylinder, cone)\n"
              << "  -s <num>   Size (for cube, plane)\n"
              << "  -h <num>   Height (for cylinder, cone)\n"
              << "  --segments <num> Number of segments (default: 32)\n"
              << "  --rings   Number of rings (for sphere, ico)\n"
              << "  --help    Show this help message\n";
}

template<typename T>
void write_mesh(const std::string& path,
               const std::vector<T>& positions,
               const std::vector<unsigned>& indices)
{
    std::string ext = get_extension(path);
    if (ext == "off") {
        Euclid::write_off<3>(path, positions, nullptr, &indices, nullptr);
    } else if (ext == "ply") {
        Euclid::write_ply<3>(path, positions, nullptr, nullptr, &indices, nullptr);
    } else if (ext == "obj") {
        Euclid::write_obj<3>(path, positions, nullptr, nullptr, &indices, nullptr);
    }
}

int main(int argc, char* argv[])
{
    std::string output_file;
    std::string primitive = "sphere";
    double radius = 1.0;
    double size = 1.0;
    double height = 2.0;
    int segments = 32;
    int rings = 16;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--sphere") == 0) {
            primitive = "sphere";
        } else if (strcmp(argv[i], "--cube") == 0) {
            primitive = "cube";
        } else if (strcmp(argv[i], "--cylinder") == 0) {
            primitive = "cylinder";
        } else if (strcmp(argv[i], "--cone") == 0) {
            primitive = "cone";
        } else if (strcmp(argv[i], "--torus") == 0) {
            primitive = "torus";
        } else if (strcmp(argv[i], "--plane") == 0) {
            primitive = "plane";
        } else if (strcmp(argv[i], "--ico") == 0) {
            primitive = "ico";
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            radius = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            size = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            height = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--segments") == 0 && i + 1 < argc) {
            segments = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--rings") == 0 && i + 1 < argc) {
            rings = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (output_file.empty()) {
        std::cerr << "Error: No output file specified\n\n";
        print_usage(argv[0]);
        return 1;
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;

    std::cout << "Generating " << primitive << "...\n";

    if (primitive == "sphere") {
        Euclid::generate_sphere(positions, indices, radius, segments, rings);
    } else if (primitive == "cube") {
        Euclid::generate_cube(positions, indices, size);
    } else if (primitive == "cylinder") {
        Euclid::generate_cylinder(positions, indices, radius, height, segments);
    } else if (primitive == "cone") {
        Euclid::generate_cone(positions, indices, radius, height, segments);
    } else if (primitive == "torus") {
        Euclid::generate_torus(positions, indices, radius, radius * 0.4, segments, rings);
    } else if (primitive == "plane") {
        Euclid::generate_plane(positions, indices, size, size);
    } else if (primitive == "ico") {
        Euclid::generate_icosphere(positions, indices, radius, rings);
    }

    std::cout << "Generated " << (positions.size() / 3) << " vertices, "
              << (indices.size() / 3) << " faces.\n";

    std::cout << "Writing mesh to " << output_file << "...\n";
    write_mesh(output_file, positions, indices);

    std::cout << "Done.\n";
    return 0;
}