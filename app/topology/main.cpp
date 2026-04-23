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
#include <Euclid/Topology/MeshTopology.h>
#include <Euclid/Topology/HomologyGenerator.h>
#include <Euclid/Topology/HomotopyGenerator.h>
#include <Euclid/Util/Color.h>

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
              << "Mesh topology analysis tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --bounds     Count number of boundaries\n"
              << "  --euler      Compute Euler characteristic\n"
              << "  --genus      Compute genus\n"
              << "  --betti      Compute Betti numbers\n"
              << "  --homology   Compute homology generators\n"
              << "  --homotopy   Compute homotopy generators\n"
              << "  --all        Compute all topology features\n"
              << "  -t <num>     Tolerance for homology/homotopy (default: 0.05)\n"
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
    bool do_bounds = false;
    bool do_euler = false;
    bool do_genus = false;
    bool do_betti = false;
    bool do_homology = false;
    bool do_homotopy = false;
    bool do_all = false;
    double tolerance = 0.05;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--bounds") == 0) {
            do_bounds = true;
        } else if (strcmp(argv[i], "--euler") == 0) {
            do_euler = true;
        } else if (strcmp(argv[i], "--genus") == 0) {
            do_genus = true;
        } else if (strcmp(argv[i], "--betti") == 0) {
            do_betti = true;
        } else if (strcmp(argv[i], "--homology") == 0) {
            do_homology = true;
        } else if (strcmp(argv[i], "--homotopy") == 0) {
            do_homotopy = true;
        } else if (strcmp(argv[i], "--all") == 0) {
            do_all = true;
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            tolerance = std::atof(argv[++i]);
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

    if (do_all) {
        do_bounds = do_euler = do_genus = do_betti = do_homology = true;
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;

    std::cout << "Reading mesh from " << input_file << "...\n";
    read_mesh(input_file, positions, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    std::cout << "Loaded " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces.\n";

    if (do_bounds || do_all) {
        int boundaries = Euclid::num_boundaries(mesh);
        std::cout << "Boundaries: " << boundaries << "\n";
    }

    if (do_euler || do_all) {
        int euler = Euclid::euler_characteristic(mesh);
        std::cout << "Euler characteristic: " << euler << "\n";
    }

    if (do_genus || do_all) {
        int g = Euclid::genus(mesh);
        std::cout << "Genus: " << g << "\n";
    }

    if (do_betti || do_all) {
        auto betti = Euclid::betti_numbers(mesh);
        std::cout << "Betti numbers: ";
        for (size_t i = 0; i < betti.size(); ++i) {
            std::cout << "b" << i << "=" << betti[i];
            if (i < betti.size() - 1) std::cout << ", ";
        }
        std::cout << "\n";
    }

    if (do_homology) {
        std::cout << "Computing homology generators (tolerance=" << tolerance << ")...\n";
        auto generators = Euclid::greedy_homology_generators(mesh, tolerance);
        std::cout << "Homology generators: " << generators.size() << "\n";
        for (size_t i = 0; i < generators.size(); ++i) {
            std::cout << "  Generator " << i << ": " << generators[i].size() << " vertices\n";
        }
    }

    if (do_homotopy) {
        std::cout << "Computing homotopy generators (tolerance=" << tolerance << ")...\n";
        auto generators = Euclid::shortest_homotopy_generators(mesh, tolerance);
        std::cout << "Homotopy generators: " << generators.size() << "\n";
        for (size_t i = 0; i < generators.size(); ++i) {
            std::cout << "  Generator " << i << ": " << generators[i].size() << " vertices\n";
        }
    }

    std::cout << "Done.\n";
    return 0;
}