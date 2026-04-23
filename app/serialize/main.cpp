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
#include <Euclid/Util/Serialize.h>
#include <Euclid/Geometry/Spectral.h>

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
              << "Mesh serialization/deserialization tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh/serialized file\n"
              << "  -o <file>    Output file\n"
              << "  --serialize  Serialize mesh with data\n"
              << "  --deserialize Deserialize and output mesh\n"
              << "  --mesh      Include mesh geometry\n"
              << "  --spectral  Include spectral data\n"
              << "  --curvature Include curvature data\n"
              << "  --format <f> Output format: bin (default), xml, json\n"
              << "  --help      Show this help message\n";

    std::cout << "\nSupported output formats:\n";
    std::cout << "  bin  - Binary (fast, compact)\n";
    std::cout << "  xml  - XML\n";
    std::cout << "  json - JSON\n";
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_serialize = false;
    bool do_deserialize = false;
    bool do_mesh = true;
    bool do_spectral = false;
    bool do_curvature = false;
    std::string format = "bin";

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--serialize") == 0) {
            do_serialize = true;
        } else if (strcmp(argv[i], "--deserialize") == 0) {
            do_deserialize = true;
        } else if (strcmp(argv[i], "--mesh") == 0) {
            do_mesh = true;
        } else if (strcmp(argv[i], "--spectral") == 0) {
            do_spectral = true;
        } else if (strcmp(argv[i], "--curvature") == 0) {
            do_curvature = true;
        } else if (strcmp(argv[i], "--format") == 0 && i + 1 < argc) {
            format = argv[++i];
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

    if (do_serialize) {
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

        Mesh mesh;
        Euclid::make_mesh<3>(mesh, positions, indices);
        auto nv = mesh.number_of_vertices();

        std::cout << "Serializing mesh (";
        if (do_mesh) std::cout << "mesh ";
        if (do_spectral) std::cout << "spectral ";
        if (do_curvature) std::cout << "curvature ";
        std::cout << ")...\n";

        if (format == "bin") {
            std::string out = output_file.empty() ? "mesh.bin" : output_file;
            Euclid::serialize(out, positions, indices);

            if (do_spectral) {
                Vec lambdas;
                Mat phis;
                Euclid::spectrum(mesh, 100, lambdas, phis, Euclid::SpecOp::mesh_laplacian);
                Euclid::serialize(out, lambdas, phis);
            }

            if (do_curvature) {
                auto gaussian = Euclid::gaussian_curvatures(mesh);
                auto mean = Euclid::mean_curvatures(mesh);
                Euclid::serialize(out, gaussian, mean);
            }

            std::cout << "Serialized to " << out << "\n";
        } else {
            std::cout << "XML/JSON serialization requires cereal built with XML/JSON support.\n";
            std::cout << "Using binary format instead.\n";
            std::string out = output_file.empty() ? "mesh.bin" : output_file;
            Euclid::serialize(out, positions, indices);
            std::cout << "Serialized to " << out << "\n";
        }
    }

    if (do_deserialize) {
        std::cout << "Deserializing from " << input_file << "...\n";

        std::vector<double> positions;
        std::vector<unsigned> indices;
        Euclid::deserialize(input_file, positions, indices);

        std::cout << "Loaded " << (positions.size() / 3) << " vertices, "
                  << (indices.size() / 3) << " faces.\n";

        if (!output_file.empty()) {
            std::string ext = get_extension(output_file);
            if (ext == "off") {
                Euclid::write_off<3>(output_file, positions, nullptr, &indices, nullptr);
            } else if (ext == "ply") {
                Euclid::write_ply<3>(output_file, positions, nullptr, nullptr, &indices, nullptr);
            } else if (ext == "obj") {
                Euclid::write_obj<3>(output_file, positions, nullptr, nullptr, &indices, nullptr);
            }
            std::cout << "Saved to " << output_file << "\n";
        }
    }

    std::cout << "Done.\n";
    return 0;
}