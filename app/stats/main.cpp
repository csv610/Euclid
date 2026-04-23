#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Aff_transformations.h>
#include <Eigen/Core>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Math/Statistics.h>
#include <Euclid/Math/Transformation.h>
#include <Euclid/Math/Numeric.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
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
              << "Statistics and transformation computation tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --stats      Compute mesh statistics\n"
              << "  --pca       Compute PCA axes\n"
              << "  --align     Align mesh to coordinate system\n"
              << "  --center    Center mesh at origin\n"
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

void compute_mesh_stats(const std::vector<double>& positions,
                        const std::vector<unsigned>& indices)
{
    auto nv = positions.size() / 3;
    Vec x(nv), y(nv), z(nv);

    for (size_t i = 0; i < nv; ++i) {
        x(i) = positions[i * 3 + 0];
        y(i) = positions[i * 3 + 1];
        z(i) = positions[i * 3 + 2];
    }

    std::cout << "Position Statistics:\n";
    std::cout << "  X: mean=" << x.mean() << ", std=" << sqrt(Euclid::variance(x)) << "\n";
    std::cout << "  Y: mean=" << y.mean() << ", std=" << sqrt(Euclid::variance(y)) << "\n";
    std::cout << "  Z: mean=" << z.mean() << ", std=" << sqrt(Euclid::variance(z)) << "\n";

    Eigen::MatrixXd points(nv, 3);
    for (size_t i = 0; i < nv; ++i) {
        points.row(i) = Eigen::Vector3d(positions[i * 3 + 0],
                                     positions[i * 3 + 1],
                                     positions[i * 3 + 2]);
    }

    Eigen::MatrixXd cov(3, 3);
    Euclid::covariance_matrix(points, cov);

    std::cout << "Covariance Matrix:\n";
    std::cout << cov << "\n";
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_stats = false;
    bool do_pca = false;
    bool do_align = false;
    bool do_center = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--stats") == 0) {
            do_stats = true;
        } else if (strcmp(argv[i], "--pca") == 0) {
            do_pca = true;
        } else if (strcmp(argv[i], "--align") == 0) {
            do_align = true;
        } else if (strcmp(argv[i], "--center") == 0) {
            do_center = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (!do_stats && !do_pca && !do_align && !do_center) {
        do_stats = true;
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

    auto nv = positions.size() / 3;
    std::cout << "Loaded " << nv << " vertices, "
              << (indices.size() / 3) << " faces.\n";

    if (do_stats) {
        compute_mesh_stats(positions, indices);
    }

    if (do_center) {
        std::cout << "Centering mesh...\n";
        double cx = 0, cy = 0, cz = 0;
        for (size_t i = 0; i < nv; ++i) {
            cx += positions[i * 3 + 0];
            cy += positions[i * 3 + 1];
            cz += positions[i * 3 + 2];
        }
        cx /= nv; cy /= nv; cz /= nv;

        for (size_t i = 0; i < nv; ++i) {
            positions[i * 3 + 0] -= cx;
            positions[i * 3 + 1] -= cy;
            positions[i * 3 + 2] -= cz;
        }

        if (!output_file.empty()) {
            std::cout << "Writing centered mesh to " << output_file << "...\n";
            write_mesh(output_file, positions, indices);
        }
    }

    std::cout << "Done.\n";
    return 0;
}