#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Eigen/Core>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Geometry/TriMeshGeometry.h>
#include <Euclid/Geometry/Spectral.h>
#include <Euclid/Topology/MeshTopology.h>
#include <Euclid/Descriptor/HKS.h>
#include <Euclid/Util/Color.h>
#include <Euclid/Util/Timer.h>

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
              << "Full shape analysis pipeline tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output directory for results\n"
              << "  --full     Run full analysis pipeline\n"
              << "  --basic   Basic geometric properties\n"
              << "  --spectral Spectral analysis\n"
              << "  --topology Topological features\n"
              << "  --descriptor Compute shape descriptors\n"
              << "  --all     Run all analyses and save everything\n"
              << "  -k <num>   Number of eigenvalues (default: 300)\n"
              << "  --help    Show this help message\n";
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
    std::string output_dir;
    bool do_full = false;
    bool do_basic = false;
    bool do_spectral = false;
    bool do_topology = false;
    bool do_descriptor = false;
    bool do_all = false;
    int k = 300;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (strcmp(argv[i], "--full") == 0) {
            do_full = true;
        } else if (strcmp(argv[i], "--basic") == 0) {
            do_basic = true;
        } else if (strcmp(argv[i], "--spectral") == 0) {
            do_spectral = true;
        } else if (strcmp(argv[i], "--topology") == 0) {
            do_topology = true;
        } else if (strcmp(argv[i], "--descriptor") == 0) {
            do_descriptor = true;
        } else if (strcmp(argv[i], "--all") == 0) {
            do_all = true;
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            k = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (do_all) {
        do_basic = do_spectral = do_topology = do_descriptor = true;
    }

    if (input_file.empty()) {
        std::cerr << "Error: No input file specified\n\n";
        print_usage(argv[0]);
        return 1;
    }

    std::vector<double> positions;
    std::vector<unsigned> indices;

    Euclid::Timer total_timer;
    total_timer.start();

    std::cout << "Reading mesh from " << input_file << "...\n";
    read_mesh(input_file, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    auto nv = mesh.number_of_vertices();
    auto nf = mesh.number_of_faces();
    std::cout << "Loaded " << nv << " vertices, " << nf << " faces.\n";

    std::string base_name = input_file;
    auto pos = base_name.rfind('/');
    if (pos != std::string::npos) {
        base_name = base_name.substr(pos + 1);
    }
    pos = base_name.rfind('.');
    if (pos != std::string::npos) {
        base_name = base_name.substr(0, pos);
    }

    std::cout << "\n=== Shape Analysis: " << base_name << " ===\n";

    if (do_basic) {
        std::cout << "\n--- Basic Geometry ---\n";
        double area = Euclid::surface_area(mesh);
        double volume = Euclid::signed_volume(mesh);
        std::cout << "Surface area: " << area << "\n";
        std::cout << "Signed volume: " << volume << "\n";

        auto gaussian = Euclid::gaussian_curvatures(mesh);
        double min_g = *std::min_element(gaussian.begin(), gaussian.end());
        double max_g = *std::max_element(gaussian.begin(), gaussian.end());
        std::cout << "Gaussian curvature: [" << min_g << ", " << max_g << "]\n";

        auto mean = Euclid::mean_curvatures(mesh);
        double min_m = *std::min_element(mean.begin(), mean.end());
        double max_m = *std::max_element(mean.begin(), mean.end());
        std::cout << "Mean curvature: [" << min_m << ", " << max_m << "]\n";

        std::vector<unsigned char> colors;
        Euclid::colormap(igl::COLOR_MAP_TYPE_JET, gaussian, colors, true);
        std::string outfile = output_dir + "/" + base_name + "_gaussian.ply";
        Euclid::write_ply<3>(outfile, positions, nullptr, &indices, &colors);

        Euclid::colormap(igl::COLOR_MAP_TYPE_JET, mean, colors, true);
        outfile = output_dir + "/" + base_name + "_mean.ply";
        Euclid::write_ply<3>(outfile, positions, nullptr, &indices, &colors);
    }

    if (do_topology) {
        std::cout << "\n--- Topology ---\n";
        int boundaries = Euclid::num_boundaries(mesh);
        int euler = Euclid::euler_characteristic(mesh);
        int genus = Euclid::genus(mesh);
        std::cout << "Boundaries: " << boundaries << "\n";
        std::cout << "Euler characteristic: " << euler << "\n";
        std::cout << "Genus: " << genus << "\n";

        auto betti = Euclid::betti_numbers(mesh);
        std::cout << "Betti numbers: ";
        for (size_t i = 0; i < betti.size(); ++i) {
            std::cout << "b" << i << "=" << betti[i];
            if (i < betti.size() - 1) std::cout << ", ";
        }
        std::cout << "\n";
    }

    if (do_spectral) {
        std::cout << "\n--- Spectral Analysis ---\n";
        Vec lambdas;
        Mat phis;
        Euclid::spectrum(mesh, k, lambdas, phis, Euclid::SpecOp::mesh_laplacian);
        std::cout << "Computed " << k << " eigenvalues\n";

        std::cout << "First 10 eigenvalues:\n";
        for (int i = 0; i < std::min(10, k); ++i) {
            std::cout << "  lambda[" << i << "] = " << lambdas(i) << "\n";
        }

        for (int i = 0; i < std::min(6, k); ++i) {
            std::vector<double> eigenfunction(nv);
            Vec::Map(eigenfunction.data(), nv) = phis.col(i);
            std::vector<unsigned char> colors;
            Euclid::colormap(igl::COLOR_MAP_TYPE_JET, eigenfunction, colors, true);
            std::stringstream ss;
            ss << output_dir << "/" << base_name << "_eig_" << i << ".ply";
            Euclid::write_ply<3>(ss.str(), positions, nullptr, &indices, &colors);
        }
    }

    if (do_descriptor) {
        std::cout << "\n--- Shape Descriptors ---\n";
        Vec lambdas;
        Mat phis;
        Euclid::spectrum(mesh, k, lambdas, phis, Euclid::SpecOp::mesh_laplacian);

        Euclid::HKS<Mesh> hks;
        hks.build(mesh, &lambdas, &phis);
        Eigen::ArrayXXd hks_sigs;
        hks.compute(hks_sigs, 100);
        std::cout << "HKS computed: " << hks_sigs.rows() << " x " << hks_sigs.cols() << "\n";

        std::vector<double> distances(nv);
        for (int i = 0; i < nv; ++i) {
            distances[i] = Euclid::chi2(hks_sigs.col(0), hks_sigs.col(i));
        }
        std::vector<unsigned char> colors;
        Euclid::colormap(igl::COLOR_MAP_TYPE_JET, distances, colors, true, false, true);
        std::string outfile = output_dir + "/" + base_name + "_hks.ply";
        Euclid::write_ply<3>(outfile, positions, nullptr, &indices, &colors);

        outfile = output_dir + "/" + base_name + "_hks.csv";
        std::ofstream ofs(outfile);
        for (int i = 0; i < hks_sigs.rows(); ++i) {
            for (int j = 0; j < hks_sigs.cols(); ++j) {
                ofs << hks_sigs(i, j);
                if (j < hks_sigs.cols() - 1) ofs << ",";
            }
            ofs << "\n";
        }
    }

    total_timer.stop();
    std::cout << "\n=== Analysis Complete in " << total_timer.elapsed() << "s ===\n";

    std::cout << "Done.\n";
    return 0;
}
