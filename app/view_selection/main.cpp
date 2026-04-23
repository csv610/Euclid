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
#include <Euclid/ViewSelection/ViewSphere.h>
#include <Euclid/ViewSelection/ProxyView.h>
#include <Euclid/ViewSelection/ViewEntropy.h>

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
              << "View selection and optimal viewpoint computation tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file (view sphere)\n"
              << "  --sphere     Compute bounding view sphere\n"
              << "  --proxy      Compute optimal proxy view\n"
              << "  --entropy    Compute view entropy\n"
              << "  --scale <n>  Scale factor for view sphere (default: 3.0)\n"
              << "  --subdiv <n> Subdivision level (default: 4)\n"
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
    bool do_sphere = true;
    bool do_proxy = false;
    bool do_entropy = false;
    double scale = 3.0;
    int subdiv = 4;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--sphere") == 0) {
            do_sphere = true;
            do_proxy = false;
            do_entropy = false;
        } else if (strcmp(argv[i], "--proxy") == 0) {
            do_proxy = true;
            do_sphere = false;
            do_entropy = false;
        } else if (strcmp(argv[i], "--entropy") == 0) {
            do_entropy = true;
            do_sphere = false;
            do_proxy = false;
        } else if (strcmp(argv[i], "--scale") == 0 && i + 1 < argc) {
            scale = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--subdiv") == 0 && i + 1 < argc) {
            subdiv = std::atoi(argv[++i]);
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
    read_mesh(input_file, positions, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    std::cout << "Loaded " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces.\n";

    std::vector<double> spositions;
    std::vector<unsigned> sindices;

    if (do_sphere) {
        std::cout << "Computing view sphere (scale=" << scale
                  << ", subdiv=" << subdiv << ")...\n";
        auto sphere = Euclid::ViewSphere<Mesh>::make_subdiv(mesh, scale, subdiv);

        std::cout << "View sphere center: ("
                  << sphere.center.x() << ", "
                  << sphere.center.y() << ", "
                  << sphere.center.z() << ")\n";
        std::cout << "View sphere radius: " << sphere.radius << "\n";

        Euclid::extract_mesh<3>(sphere.mesh, spositions, &sindices);
    }

    if (do_proxy) {
        std::cout << "Computing optimal proxy view...\n";
        auto views = Euclid::ProxyView<Mesh>::compute(mesh, 16);

        std::cout << "Found " << views.size() << " proxy views:\n";
        for (size_t i = 0; i < views.size(); ++i) {
            std::cout << "  View " << i << ": ("
                      << views[i].first[0] << ", "
                      << views[i].first[1] << ", "
                      << views[i].first[2] << ")";
            std::cout << " score=" << views[i].second << "\n";
        }

        if (!views.empty()) {
            Euclid::extract_mesh<3>(views[0].first, spositions, &sindices);
        }
    }

    if (do_entropy) {
        std::cout << "Computing view entropy...\n";
        auto sphere = Euclid::ViewSphere<Mesh>::make_subdiv(mesh, scale, subdiv);
        auto entropy = Euclid::ViewEntropy<Mesh>::compute(mesh, sphere);

        std::cout << "View entropy computed. Best views:\n";
        for (size_t i = 0; i < std::min(5, (int)entropy.size()); ++i) {
            std::cout << "  View " << i << ": entropy=" << entropy[i].first
                      << " ("
                      << entropy[i].second[0] << ", "
                      << entropy[i].second[1] << ", "
                      << entropy[i].second[2] << ")\n";
        }

        if (!entropy.empty()) {
            Mesh vmesh;
            auto view_point = entropy[0].second;
            Euclid::extract_mesh<3>(vmesh, spositions, &sindices);
        }
    }

    if (!output_file.empty() && !spositions.empty()) {
        std::cout << "Writing view sphere mesh to " << output_file << "...\n";
        Euclid::write_off<3>(output_file, spositions, nullptr, &sindices, nullptr);
    }

    std::cout << "Done.\n";
    return 0;
}