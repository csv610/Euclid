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
#include <Euclid/Geometry/TriMeshGeometry.h>
#include <Euclid/Geometry/Spectral.h>
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
              << "Geometry processing and spectral analysis tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --curvature  Compute and export curvature (mean, gaussian)\n"
              << "  --spectral   Compute spectral decomposition (eigenfunctions)\n"
              << "  --hks        Compute Heat Kernel Signature\n"
              << "  -- normals   Compute vertex normals\n"
              << "  --area       Compute mesh surface area\n"
              << "  --volume     Compute mesh volume\n"
              << "  -k <num>     Number of eigenvalues for spectral analysis (default: 50)\n"
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
                const std::vector<T>* normals,
                const std::vector<unsigned>& indices,
                const std::vector<unsigned char>* colors)
{
    std::string ext = get_extension(path);
    if (ext == "off") {
        Euclid::write_off<3>(path, positions, normals, &indices, nullptr);
    } else if (ext == "ply") {
        Euclid::write_ply<3>(path, positions, normals, nullptr, &indices, colors);
    } else if (ext == "obj") {
        Euclid::write_obj<3>(path, positions, normals, nullptr, &indices, colors);
    }
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_curvature = false;
    bool do_spectral = false;
    bool do_hks = false;
    bool do_normals = false;
    bool do_area = false;
    bool do_volume = false;
    int k = 50;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--curvature") == 0) {
            do_curvature = true;
        } else if (strcmp(argv[i], "--spectral") == 0) {
            do_spectral = true;
        } else if (strcmp(argv[i], "--hks") == 0) {
            do_hks = true;
        } else if (strcmp(argv[i], "--normals") == 0) {
            do_normals = true;
        } else if (strcmp(argv[i], "--area") == 0) {
            do_area = true;
        } else if (strcmp(argv[i], "--volume") == 0) {
            do_volume = true;
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

    std::vector<double> positions;
    std::vector<double> normals;
    std::vector<unsigned> indices;

    std::cout << "Reading mesh from " << input_file << "...\n";
    read_mesh(input_file, positions, &normals, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    std::cout << "Loaded " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces.\n";

    std::vector<unsigned char> colors;

    if (do_area) {
        double area = Euclid::surface_area(mesh);
        std::cout << "Surface area: " << area << "\n";
    }

    if (do_volume) {
        double volume = Euclid::signed_volume(mesh);
        std::cout << "Signed volume: " << volume << "\n";
    }

    if (do_normals) {
        std::cout << "Computing vertex normals...\n";
        normals.clear();
        Euclid::vertex_normals(mesh, normals);
        std::cout << "Vertex normals computed.\n";
    }

    if (do_curvature) {
        std::cout << "Computing curvatures...\n";
        auto gaussian = Euclid::gaussian_curvatures(mesh);
        auto mean = Euclid::mean_curvatures(mesh);

        std::vector<unsigned char> gaussian_colors;
        Euclid::colormap(igl::COLOR_MAP_TYPE_JET, gaussian, gaussian_colors, true);

        if (!output_file.empty()) {
            std::string gfile = output_file;
            auto pos = gfile.rfind('.');
            if (pos != std::string::npos) {
                gfile.insert(pos, "_gaussian");
            }
            std::cout << "Writing gaussian curvature to " << gfile << "...\n";
            Euclid::write_ply<3>(gfile, positions, nullptr, &indices, &gaussian_colors);

            std::vector<unsigned char> mean_colors;
            Euclid::colormap(igl::COLOR_MAP_TYPE_JET, mean, mean_colors, true);
            gfile = output_file;
            pos = gfile.rfind('.');
            if (pos != std::string::npos) {
                gfile.insert(pos, "_mean");
            }
            std::cout << "Writing mean curvature to " << gfile << "...\n";
            Euclid::write_ply<3>(gfile, positions, nullptr, &indices, &mean_colors);
        }
    }

    if (do_spectral) {
        std::cout << "Computing spectral decomposition (k=" << k << ")...\n";
        Euclid::Timer timer;
        timer.start();

        Vec lambdas;
        Mat phis;
        Euclid::spectrum(mesh, k, lambdas, phis, Euclid::SpecOp::mesh_laplacian);

        timer.stop();
        std::cout << "Spectral computation done in " << timer.elapsed() << "s\n";

        std::cout << "Eigenvalues:\n";
        for (int i = 0; i < std::min(10, k); ++i) {
            std::cout << "  lambda[" << i << "] = " << lambdas(i) << "\n";
        }

        if (!output_file.empty()) {
            for (int i = 0; i < std::min(6, k); ++i) {
                std::vector<double> eigenfunction(mesh.number_of_vertices());
                Vec::Map(eigenfunction.data(), mesh.number_of_vertices()) = phis.col(i);

                std::vector<unsigned char> eig_colors;
                Euclid::colormap(igl::COLOR_MAP_TYPE_JET, eigenfunction, eig_colors, true);

                std::stringstream ss;
                ss << output_file;
                auto pos = ss.str().rfind('.');
                if (pos != std::string::npos) {
                    ss.str(ss.str().insert(pos, "_eig_" + std::to_string(i)));
                }
                std::cout << "Writing eigenfunction " << i << " to " << ss.str() << "...\n";
                Euclid::write_ply<3>(ss.str(), positions, nullptr, &indices, &eig_colors);
            }
        }
    }

    if (do_hks) {
        std::cout << "Computing Heat Kernel Signature...\n";
        Euclid::Timer timer;
        timer.start();

        Vec lambdas;
        Mat phis;
        Euclid::spectrum(mesh, k, lambdas, phis, Euclid::SpecOp::mesh_laplacian);

        Euclid::HKS<Mesh> hks;
        hks.build(mesh, &lambdas, &phis);

        Eigen::ArrayXXd hks_sigs;
        hks.compute(hks_sigs, 100);

        timer.stop();
        std::cout << "HKS computation done in " << timer.elapsed() << "s\n";

        std::cout << "HKS signature dimensions: " << hks_sigs.rows() << " vertices x "
                  << hks_sigs.cols() << " time steps\n";

        if (!output_file.empty()) {
            std::vector<double> distances(mesh.number_of_vertices());
            for (int i = 0; i < mesh.number_of_vertices(); ++i) {
                distances[i] = Euclid::chi2(hks_sigs.col(0), hks_sigs.col(i));
            }

            std::vector<unsigned char> hks_colors;
            Euclid::colormap(igl::COLOR_MAP_TYPE_JET, distances, hks_colors, true, false, true);

            std::string hksfile = output_file;
            auto pos = hksfile.rfind('.');
            if (pos != std::string::npos) {
                hksfile.insert(pos, "_hks");
            }
            std::cout << "Writing HKS to " << hksfile << "...\n";
            Euclid::write_ply<3>(hksfile, positions, nullptr, &indices, &hks_colors);
        }
    }

    if (!output_file.empty() && !do_curvature && !do_spectral && !do_hks) {
        std::cout << "Writing mesh to " << output_file << "...\n";
        write_mesh(output_file, positions, normals.empty() ? nullptr : &normals,
                   indices, nullptr);
    }

    std::cout << "Done.\n";
    return 0;
}
