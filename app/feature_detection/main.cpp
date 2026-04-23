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
#include <Euclid/Geometry/TriMeshGeometry.h>
#include <Euclid/FeatureDetection/NativeHKS.h>
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
              << "Feature detection on meshes\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --critical  Detect critical points (peaks, pits, ridges)\n"
              << "  --saliency Compute mesh saliency\n"
              << "  --ridges   Detect sharp edges/ridges\n"
              << "  --hks     Heat Kernel Signature feature detection\n"
              << "  --corners  Detect corners/vertices\n"
              << "  --threshold <n> Detection threshold (default: 0.5)\n"
              >> "  --help     Show this help message\n";
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_critical = false;
    bool do_saliency = false;
    bool do_ridges = false;
    bool do_hks = false;
    bool do_corners = false;
    double threshold = 0.5;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--critical") == 0) {
            do_critical = true;
        } else if (strcmp(argv[i], "--saliency") == 0) {
            do_saliency = true;
        } else if (strcmp(argv[i], "--ridges") == 0) {
            do_ridges = true;
        } else if (strcmp(argv[i], "--hks") == 0) {
            do_hks = true;
        } else if (strcmp(argv[i], "--corners") == 0) {
            do_corners = true;
        } else if (strcmp(argv[i], "--threshold") == 0 && i + 1 < argc) {
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

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    std::cout << "Loaded " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces.\n";

    std::vector<unsigned char> colors;

    if (do_critical) {
        std::cout << "Detecting critical points...\n";
        auto gaussian = Euclid::gaussian_curvatures(mesh);
        std::vector<int> peaks, pits, ridges;
        for (size_t i = 0; i < gaussian.size(); ++i) {
            if (gaussian[i] > threshold) peaks.push_back(i);
            else if (gaussian[i] < -threshold) pits.push_back(i);
        }
        std::cout << "Found " << peaks.size() << " peaks, " << pits.size() << " pits\n";

        colors.resize(mesh.number_of_vertices() * 4, 128);
        for (int idx : peaks) {
            colors[4 * idx + 0] = 255;
            colors[4 * idx + 1] = 0;
            colors[4 * idx + 2] = 0;
            colors[4 * idx + 3] = 255;
        }
        for (int idx : pits) {
            colors[4 * idx + 0] = 0;
            colors[4 * idx + 1] = 0;
            colors[4 * idx + 2] = 255;
            colors[4 * idx + 3] = 255;
        }
    }

    if (do_saliency) {
        std::cout << "Computing mesh saliency...\n";
        auto gaussian = Euclid::gaussian_curvatures(mesh);
        auto mean = Euclid::mean_curvatures(mesh);
        std::vector<double> saliency(mesh.number_of_vertices());
        for (size_t i = 0; i < saliency.size(); ++i) {
            saliency[i] = std::abs(gaussian[i]) + std::abs(mean[i]);
        }
        Euclid::colormap(igl::COLOR_MAP_TYPE_JET, saliency, colors, true);

        if (!output_file.empty()) {
            std::string sfile = output_file;
            auto pos = sfile.rfind('.');
            if (pos != std::string::npos) {
                sfile.insert(pos, "_saliency");
            }
            std::cout << "Writing saliency to " << sfile << "...\n";
            Euclid::write_ply<3>(sfile, positions, nullptr, &indices, &colors);
        }
    }

    if (do_ridges) {
        std::cout << "Detecting ridges/sharp edges...\n";
        auto gaussian = Euclid::gaussian_curvatures(mesh);
        auto mean = Euclid::mean_curvatures(mesh);
        std::vector<double> ridges;
        for (size_t i = 0; i < gaussian.size(); ++i) {
            if (std::abs(mean[i]) > threshold) {
                ridges.push_back(i);
            }
        }
        std::cout << "Found " << ridges.size() << " ridge vertices\n";

        colors.assign(mesh.number_of_vertices() * 4, 200);
        for (size_t i = 0; i < ridges.size(); ++i) {
            colors[4 * ridges[i] + 0] = 255;
            colors[4 * ridges[i] + 1] = 255;
            colors[4 * ridges[i] + 2] = 0;
            colors[4 * ridges[i] + 3] = 255;
        }
    }

    if (do_corners) {
        std::cout << "Detecting corners...\n";
        auto gaussian = Euclid::gaussian_curvatures(mesh);
        std::vector<int> corners;
        for (size_t i = 0; i < gaussian.size(); ++i) {
            if (std::abs(gaussian[i]) > threshold) {
                corners.push_back(i);
            }
        }
        std::cout << "Found " << corners.size() << " corner vertices\n";

        colors.assign(mesh.number_of_vertices() * 4, 200);
        for (int idx : corners) {
            colors[4 * idx + 0] = 255;
            colors[4 * idx + 1] = 255;
            colors[4 * idx + 2] = 0;
            colors[4 * idx + 3] = 255;
        }
    }

    if (!output_file.empty() && !colors.empty()) {
        std::cout << "Writing to " << output_file << "...\n";
        Euclid::write_ply<3>(output_file, positions, nullptr, &indices, &colors);
    }

    std::cout << "Done.\n";
    return 0;
}