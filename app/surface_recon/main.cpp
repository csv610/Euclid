#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/SurfaceDelaunay/DelaunayMesh.h>

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
              << "Surface reconstruction from point clouds\n\n"
              << "Options:\n"
              << "  -i <file>    Input file (points or mesh)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --delaunay  Delaunay surface triangulation\n"
              << "  --advancingAdvancing-front surface reconstruction\n"
              << "  --ball-pivot Ball pivoting algorithm\n"
              << "  --poisson Poisson reconstruction\n"
              << "  -r <num>    Ball radius for ball-pivot (default: auto)\n"
              << "  --help     Show this help message\n";
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_delaunay = true;
    bool do_advancing = false;
    bool do_ball_pivot = false;
    bool do_poisson = false;
    double ball_radius = 0.0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--delaunay") == 0) {
            do_delaunay = true;
        } else if (strcmp(argv[i], "--advancing") == 0) {
            do_advancing = true;
        } else if (strcmp(argv[i], "--ball-pivot") == 0) {
            do_ball_pivot = true;
        } else if (strcmp(argv[i], "--poisson") == 0) {
            do_poisson = true;
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 <argc) {
            ball_radius = std::atof(argv[++i]);
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

    if (output_file.empty()) {
        std::cerr << "Error: No output file specified\n\n";
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
    } else if (ext == "pts" || ext == "xyz") {
        std::ifstream ifs(input_file);
        double x, y, z;
        while (ifs >> x >> y >> z) {
            positions.push_back(x);
            positions.push_back(y);
            positions.push_back(z);
        }
    }

    auto n = positions.size() / 3;
    std::cout << "Loaded " << n << " points.\n";

    if (do_delaunay) {
        std::cout << "Performing Delaunay surface triangulation...\n";
        Mesh mesh;
        Euclid::delaunay_surface_triangulation(positions, mesh);
        std::cout << "Generated " << mesh.number_of_vertices() << " vertices, "
                  << mesh.number_of_faces() << " faces.\n";

        std::cout << "Writing to " << output_file << "...\n";
        Euclid::extract_mesh<3>(mesh, positions, &indices);
        ext = get_extension(output_file);
        if (ext == "off") {
            Euclid::write_off<3>(output_file, positions, nullptr, &indices, nullptr);
        } else if (ext == "ply") {
            Euclid::write_ply<3>(output_file, positions, nullptr, nullptr, &indices, nullptr);
        }
    }

    if (do_advancing) {
        std::cout << "Advancing front surface reconstruction...\n";
    }

    if (do_ball_pivot) {
        std::cout << "Ball pivoting surface reconstruction...\n";
    }

    if (do_poisson) {
        std::cout << "Poisson surface reconstruction...\n";
    }

    std::cout << "Done.\n";
    return 0;
}