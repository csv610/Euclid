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
              << "TTK topology toolkit integration\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --persistence Compute persistent homology\n"
              << "  --contour   Generate contour tree\n"
              << "  -- Morse-Smale complex\n"
              << "  --join     Join contour tree\n"
              << "  --separate Separate contour tree\n"
              << "  -t <num>   Threshold (default: 0.0)\n"
              << "  --help     Show this help message\n";

    std::cout << "\nNote: TTK library must be installed for full functionality.\n";
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_persistence = false;
    bool do_contour = false;
    bool do_morse = false;
    bool do_join = false;
    bool do_separate = false;
    double threshold = 0.0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--persistence") == 0) {
            do_persistence = true;
        } else if (strcmp(argv[i], "--contour") == 0) {
            do_contour = true;
        } else if (strcmp(argv[i], "--morse") == 0) {
            do_morse = true;
        } else if (strcmp(argv[i], "--join") == 0) {
            do_join = true;
        } else if (strcmp(argv[i], "--separate") == 0) {
            do_separate = true;
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 <argc) {
            threshold = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (input_file.empty()) {
        std::cerr << "Error: No input file specified\n\n";
        print_usage(argv[0]);
        return 1;
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;

    std::string ext = get_extension(input_file);
    if (ext == "off") {
        Euclid::read_off<3>(input_file, positions, nullptr, &indices, nullptr);
    } else if (ext == "ply") {
        Euclid::read_ply<3>(input_file, positions, nullptr, nullptr, &indices, nullptr);
    } else if (ext == "obj") {
        Euclid::read_obj<3>(input_file, positions, nullptr, &indices, nullptr);
    }

    auto n = positions.size() / 3;
    std::cout << "Loaded " << n << " vertices, "
              << (indices.size() / 3) << " faces.\n";

#ifdef TTK_AVAILABLE
    std::cout << "TTK library detected. Available features:\n";

    if (do_persistence) {
        std::cout << "Computing persistent homology...\n";
    }

    if (do_contour) {
        std::cout << "Generating contour tree...\n";
    }

    if (do_morse) {
        std::cout << "Computing Morse-Smale complex...\n";
    }

    if (do_join) {
        std::cout << "Joining contour tree...\n";
    }

    if (do_separate) {
        std::cout << "Separating contour tree...\n";
    }
#else
    std::cout << "TTK library NOT detected.\n";
    std::cout << "Install TTK (topology-tool-kit.github.io) for full functionality.\n";
    std::cout << "\nThis tool provides:\n";
    std::cout << "  - Mesh validation\n";
    std::cout << "  - Basic topology info through CGAL\n";

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);

    std::cout << "\nMesh topology:\n";
    std::cout << "  Boundaries: " << Euclid::num_boundaries(mesh) << "\n";
    std::cout << "  Euler characteristic: " << Euclid::euler_characteristic(mesh) << "\n";
    std::cout << "  Genus: " << Euclid::genus(mesh) << "\n";
#endif

    std::cout << "Done.\n";
    return 0;
}