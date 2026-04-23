#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <algorithm>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Eigen/Core>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Geometry/Spectral.h>
#include <Euclid/Descriptor/HKS.h>
#include <Euclid/Descriptor/WKS.h>
#include <Euclid/Descriptor/SpinImage.h>
#include <Euclid/Descriptor/Histogram.h>
#include <Euclid/Util/Color.h>
#include <Euclid/Util/Timer.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
using Arr = Eigen::ArrayXXd;

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
              << "Shape descriptor and feature computation tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --hks        Compute Heat Kernel Signature\n"
              << "  --wks        Compute Wave Kernel Signature\n"
              << "  --spin       Compute Spin Image\n"
              << "  --histogram  Compute curvature histogram\n"
              << "  --match      Perform shape matching\n"
              << "  -k <num>     Number of eigenvalues (default: 300)\n"
              << "  -t <num>     Number of time steps (default: 100)\n"
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
    bool do_hks = false;
    bool do_wks = false;
    bool do_spin = false;
    bool do_histogram = false;
    bool do_match = false;
    int k = 300;
    int timestep = 100;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--hks") == 0) {
            do_hks = true;
        } else if (strcmp(argv[i], "--wks") == 0) {
            do_wks = true;
        } else if (strcmp(argv[i], "--spin") == 0) {
            do_spin = true;
        } else if (strcmp(argv[i], "--histogram") == 0) {
            do_histogram = true;
        } else if (strcmp(argv[i], "--match") == 0) {
            do_match = true;
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            k = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            timestep = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (!do_hks && !do_wks && !do_spin && !do_histogram) {
        do_hks = true;
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

    std::vector<unsigned char> colors;

    if (do_hks || do_wks || do_match) {
        std::cout << "Computing spectral basis (k=" << k << ")...\n";
        Euclid::Timer timer;
        timer.start();

        Vec lambdas;
        Mat phis;
        Euclid::spectrum(mesh, k, lambdas, phis, Euclid::SpecOp::mesh_laplacian);

        timer.stop();
        std::cout << "Spectral computation done in " << timer.elapsed() << "s\n";

        if (do_hks) {
            std::cout << "Computing Heat Kernel Signature (timesteps=" << timestep << ")...\n";
            timer.start();

            Euclid::HKS<Mesh> hks;
            hks.build(mesh, &lambdas, &phis);

            Arr hks_sigs;
            hks.compute(hks_sigs, timestep);

            timer.stop();
            std::cout << "HKS computation done in " << timer.elapsed() << "s\n";

            std::vector<double> distances(mesh.number_of_vertices());
            for (int i = 0; i < mesh.number_of_vertices(); ++i) {
                distances[i] = Euclid::chi2(hks_sigs.col(0), hks_sigs.col(i));
            }

            Euclid::colormap(igl::COLOR_MAP_TYPE_JET, distances, colors, true, false, true);

            if (!output_file.empty()) {
                std::string hksfile = output_file;
                auto pos = hksfile.rfind('.');
                if (pos != std::string::npos) {
                    hksfile.insert(pos, "_hks");
                }
                std::cout << "Writing HKS to " << hksfile << "...\n";
                Euclid::write_ply<3>(hksfile, positions, nullptr, &indices, &colors);
            }
        }

        if (do_wks) {
            std::cout << "Computing Wave Kernel Signature (energy steps=" << timestep << ")...\n";
            timer.start();

            Euclid::WKS<Mesh> wks;
            wks.build(mesh, &lambdas, &phis);

            Arr wks_sigs;
            wks.compute(wks_sigs, timestep);

            timer.stop();
            std::cout << "WKS computation done in " << timer.elapsed() << "s\n";

            std::vector<double> distances(mesh.number_of_vertices());
            for (int i = 0; i < mesh.number_of_vertices(); ++i) {
                distances[i] = Euclid::chi2(wks_sigs.col(0), wks_sigs.col(i));
            }

            Euclid::colormap(igl::COLOR_MAP_TYPE_JET, distances, colors, true, false, true);

            if (!output_file.empty() && !do_hks) {
                std::string wksfile = output_file;
                auto pos = wksfile.rfind('.');
                if (pos != std::string::npos) {
                    wksfile.insert(pos, "_wks");
                }
                std::cout << "Writing WKS to " << wksfile << "...\n";
                Euclid::write_ply<3>(wksfile, positions, nullptr, &indices, &colors);
            }
        }
    }

    if (do_spin) {
        std::cout << "Computing Spin Images...\n";
        Euclid::Timer timer;
        timer.start();

        Euclid::SpinImage<Mesh> spin;
        auto descriptors = spin.build(mesh);

        timer.stop();
        std::cout << "Spin Image computation done in " << timer.elapsed() << "s\n";
        std::cout << "Descriptors computed for " << descriptors.size() << " vertices\n";
    }

    if (do_histogram) {
        std::cout << "Computing curvature histogram...\n";
        auto gaussian = Euclid::gaussian_curvatures(mesh);
        auto mean = Euclid::mean_curvatures(mesh);

        std::vector<double> gaussian_hist;
        std::vector<double> mean_hist;
        int nbins = 50;

        Euclid::histogram(gaussian, nbins, gaussian_hist);
        Euclid::histogram(mean, nbins, mean_hist);

        std::cout << "Gaussian curvature histogram:\n";
        for (size_t i = 0; i < gaussian_hist.size(); ++i) {
            std::cout << "  bin " << i << ": " << gaussian_hist[i] << "\n";
        }

        std::cout << "Mean curvature histogram:\n";
        for (size_t i = 0; i < mean_hist.size(); ++i) {
            std::cout << "  bin " << i << ": " << mean_hist[i] << "\n";
        }
    }

    std::cout << "Done.\n";
    return 0;
}