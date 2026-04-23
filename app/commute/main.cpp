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
#include <Euclid/Geometry/Spectral.h>
#include <Euclid/Distance/CommuteTimeDistance.h>
#include <Euclid/Util/Color.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;

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
              << "Commute time distance computation tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  -s <idx>     Source vertex index\n"
              << "  -t <idx>     Target vertex index\n"
              << "  --all        Compute distances from all vertices to target\n"
              << "  -k <num>     Number of eigenvalues (default: 300)\n"
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
    int source = -1;
    int target = -1;
    bool do_all = false;
    int k = 300;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            source = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            target = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--all") == 0) {
            do_all = true;
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            k = std::atoi(argv[++i]);
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

    if (target < 0 && source < 0) {
        std::cerr << "Error: No target/source vertex specified\n\n";
        print_usage(argv[0]);
        return 1;
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;

    std::cout << "Reading mesh from " << input_file << "...\n";
    read_mesh(input_file, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    auto nv = mesh.number_of_vertices();
    std::cout << "Loaded " << nv << " vertices, "
              << mesh.number_of_faces() << " faces.\n";

    std::cout << "Computing spectral decomposition (k=" << k << ")...\n";
    Vec lambdas;
    Mat phis;
    Euclid::spectrum(mesh, k, lambdas, phis, Euclid::SpecOp::graph_laplacian);

    if (target >= 0) {
        if (do_all) {
            std::cout << "Computing commute times from all vertices to "
                      << target << "...\n";
            std::vector<double> distances(nv);
            for (int i = 0; i < nv; ++i) {
                distances[i] = Euclid::commute_time_distance(lambdas, phis, i, target);
            }

            std::vector<unsigned char> colors;
            Euclid::colormap(igl::COLOR_MAP_TYPE_PARULA, distances, colors, true, true);

            if (!output_file.empty()) {
                std::cout << "Writing commute times to " << output_file << "...\n";
                Euclid::write_ply<3>(output_file, positions, nullptr, &indices, &colors);
            }
        } else if (source >= 0) {
            double dist = Euclid::commute_time_distance(lambdas, phis, source, target);
            std::cout << "Commute time distance between " << source
                      << " and " << target << ": " << dist << "\n";
        } else {
            std::cout << "Computing distances from all vertices to "
                      << target << "...\n";
            std::vector<double> distances(nv);
            for (int i = 0; i < nv; ++i) {
                distances[i] = Euclid::commute_time_distance(lambdas, phis, i, target);
            }

            std::cout << "Sample distances:\n";
            for (int i = 0; i < std::min(10, nv); ++i) {
                std::cout << "  To vertex " << i << ": " << distances[i] << "\n";
            }
        }
    }

    std::cout << "Done.\n";
    return 0;
}