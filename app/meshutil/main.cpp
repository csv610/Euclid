#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <fstream>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Eigen/Core>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/EigenMesh.h>
#include <Euclid/MeshUtil/MeshHelpers.h>

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
              << "Mesh utility conversion tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --to-eigen   Convert to Eigen matrix format\n"
              << "  --from-eigen Convert from Eigen matrix format\n"
              << "  --ring      Find vertex n-ring\n"
              << "  -v <idx>    Vertex index for ring computation\n"
              << "  -n <num>    Ring size (default: 1)\n"
              << "  --edges     List mesh edges\n"
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
    std::string input_file;
    std::string output_file;
    bool do_to_eigen = false;
    bool do_from_eigen = false;
    bool do_ring = false;
    bool do_edges = false;
    int ring_vidx = 0;
    int ring_n = 1;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--to-eigen") == 0) {
            do_to_eigen = true;
        } else if (strcmp(argv[i], "--from-eigen") == 0) {
            do_from_eigen = true;
        } else if (strcmp(argv[i], "--ring") == 0) {
            do_ring = true;
        } else if (strcmp(argv[i], "-v") == 0 && i + 1 < argc) {
            ring_vidx = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            ring_n = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--edges") == 0) {
            do_edges = true;
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

    std::cout << "Reading mesh from " << input_file << "...\n";
    read_mesh(input_file, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    std::cout << "Loaded " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces.\n";

    if (do_to_eigen) {
        std::cout << "Converting to Eigen matrix format...\n";
        Mat V, F;
        Euclid::make_mesh(V, F, positions, indices);

        std::cout << "V: " << V.rows() << "x" << V.cols() << "\n";
        std::cout << "F: " << F.rows() << "x" << F.cols() << "\n";

        if (!output_file.empty()) {
            std::string mfile = output_file;
            auto pos = mfile.rfind('.');
            if (pos != std::string::npos) {
                mfile.insert(pos, "_V");
            }
            std::ofstream ofs(mfile);
            ofs << "V = [\n" << V << "];\n\nF = [\n" << F << "];\n";
            std::cout << "Written to " << mfile << "\n";
        }
    }

    if (do_ring) {
        std::cout << "Finding " << ring_n << "-ring around vertex " << ring_vidx << "...\n";
        auto ring = Euclid::nring_vertices(Mesh::Vertex_index(ring_vidx), mesh, ring_n);
        std::cout << "Found " << ring.size() << " vertices in ring\n";
    }

    if (do_edges) {
        std::cout << "Counting edges...\n";
        auto edges = edges(mesh);
        std::cout << "Mesh has " << std::distance(edges.first, edges.second) << " edges\n";
    }

    std::cout << "Done.\n";
    return 0;
}