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
#include <Euclid/Distance/GeodesicsInHeat.h>
#include <Euclid/Distance/DiffusionDistance.h>
#include <Euclid/Distance/BiharmonicDistance.h>
#include <Euclid/Util/Color.h>
#include <Euclid/Util/Timer.h>

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
              << "Geodesic and diffusion distance computation tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  -s <idx>     Source vertex index (default: 0)\n"
              << "  --geodesic   Compute geodesic distance from source\n"
              << "  --diffusion  Compute diffusion distance\n"
              << "  --biharmonic Compute biharmonic distance\n"
              << "  --scale     Heat method scale factor (default: 4.0)\n"
              << "  --timestep   Diffusion timestep (default: 0.001)\n"
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
    int source = 0;
    bool do_geodesic = true;
    bool do_diffusion = false;
    bool do_biharmonic = false;
    double scale = 4.0;
    double timestep = 0.001;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            source = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--geodesic") == 0) {
            do_geodesic = true;
        } else if (strcmp(argv[i], "--diffusion") == 0) {
            do_diffusion = true;
        } else if (strcmp(argv[i], "--biharmonic") == 0) {
            do_biharmonic = true;
        } else if (strcmp(argv[i], "--scale") == 0 && i + 1 < argc) {
            scale = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--timestep") == 0 && i + 1 < argc) {
            timestep = std::atof(argv[++i]);
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

    if (source < 0 || source >= mesh.number_of_vertices()) {
        std::cerr << "Error: Invalid source vertex index " << source << "\n";
        return 1;
    }

    if (do_geodesic) {
        std::cout << "Computing geodesic distances (source=" << source
                  << ", scale=" << scale << ")...\n";
        Euclid::Timer timer;
        timer.start();

        Euclid::GeodesicsInHeat<Mesh> heat_method;
        heat_method.build(mesh, scale);

        std::vector<double> geodesics;
        heat_method.compute(Mesh::Vertex_index(source), geodesics);

        timer.stop();
        std::cout << "Geodesic computation done in " << timer.elapsed() << "s\n";

        std::vector<unsigned char> colors;
        Euclid::colormap(igl::COLOR_MAP_TYPE_PARULA, geodesics, colors, true, true);

        if (!output_file.empty()) {
            std::string gfile = output_file;
            auto pos = gfile.rfind('.');
            if (pos != std::string::npos) {
                gfile.insert(pos, "_geodesic");
            }
            std::cout << "Writing geodesic distances to " << gfile << "...\n";
            Euclid::write_ply<3>(gfile, positions, nullptr, &indices, &colors);
        }
    }

    if (do_diffusion) {
        std::cout << "Computing diffusion distances (timestep=" << timestep << ")...\n";
        Euclid::Timer timer;
        timer.start();

        Euclid::DiffusionDistance<Mesh> diff_method;
        diff_method.build(mesh, timestep);

        std::vector<double> distances(mesh.number_of_vertices());
        diff_method.compute(Mesh::Vertex_index(source), distances);

        timer.stop();
        std::cout << "Diffusion distance done in " << timer.elapsed() << "s\n";

        std::vector<unsigned char> colors;
        Euclid::colormap(igl::COLOR_MAP_TYPE_PARULA, distances, colors, true, true);

        if (!output_file.empty()) {
            std::string dfile = output_file;
            auto pos = dfile.rfind('.');
            if (pos != std::string::npos) {
                dfile.insert(pos, "_diffusion");
            }
            std::cout << "Writing diffusion distances to " << dfile << "...\n";
            Euclid::write_ply<3>(dfile, positions, nullptr, &indices, &colors);
        }
    }

    if (do_biharmonic) {
        std::cout << "Computing biharmonic distances...\n";
        Euclid::Timer timer;
        timer.start();

        Euclid::BiharmonicDistance<Mesh> bihar_method;
        bihar_method.build(mesh);

        std::vector<double> distances(mesh.number_of_vertices());
        bihar_method.compute(Mesh::Vertex_index(source), distances);

        timer.stop();
        std::cout << "Biharmonic distance done in " << timer.elapsed() << "s\n";

        std::vector<unsigned char> colors;
        Euclid::colormap(igl::COLOR_MAP_TYPE_PARULA, distances, colors, true, true);

        if (!output_file.empty()) {
            std::string bfile = output_file;
            auto pos = bfile.rfind('.');
            if (pos != std::string::npos) {
                bfile.insert(pos, "_biharmonic");
            }
            std::cout << "Writing biharmonic distances to " << bfile << "...\n";
            Euclid::write_ply<3>(bfile, positions, nullptr, &indices, &colors);
        }
    }

    std::cout << "Done.\n";
    return 0;
}