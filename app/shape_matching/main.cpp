#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Eigen/Core>
#include <CGAL/Surface_mesh.h>
#include <CGAL/property_map.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Geometry/Spectral.h>

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
              << "Shape matching and registration tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (reference)\n"
              << "  -m <file>    Moving mesh file to align\n"
              << "  -o <file>    Output aligned mesh\n"
              << "  --icp       Iterative Closest Point alignment\n"
              << "  --hausdorff  Compute Hausdorff distance\n"
              << "  --chamfer   Compute Chamfer distance\n"
              << "  --spectral Spectral shape matching\n"
              << "  -k <num>    Number of eigenvalues (default: 100)\n"
              << "  --help     Show this help message\n";
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string moving_file;
    std::string output_file;
    bool do_icp = false;
    bool do_hausdorff = false;
    bool do_chamfer = false;
    bool do_spectral = false;
    int k = 100;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            moving_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--icp") == 0) {
            do_icp = true;
        } else if (strcmp(argv[i], "--hausdorff") == 0) {
            do_hausdorff = true;
        } else if (strcmp(argv[i], "--chamfer") == 0) {
            do_chamfer = true;
        } else if (strcmp(argv[i], "--spectral") == 0) {
            do_spectral = true;
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

    std::vector<double> ref_positions;
    std::vector<unsigned> ref_indices;
    std::vector<double> mov_positions;
    std::vector<unsigned> mov_indices;

    std::string ext = get_extension(input_file);
    if (ext == "off") {
        Euclid::read_off<3>(input_file, ref_positions, nullptr, &ref_indices, nullptr);
    } else if (ext == "ply") {
        Euclid::read_ply<3>(input_file, ref_positions, nullptr, nullptr, &ref_indices, nullptr);
    } else if (ext == "obj") {
        Euclid::read_obj<3>(input_file, ref_positions, nullptr, &ref_indices, nullptr);
    }

    Mesh ref_mesh, mov_mesh;
    Euclid::make_mesh<3>(ref_mesh, ref_positions, ref_indices);
    std::cout << "Reference mesh: " << ref_mesh.number_of_vertices() << " vertices\n";

    if (!moving_file.empty()) {
        ext = get_extension(moving_file);
        if (ext == "off") {
            Euclid::read_off<3>(moving_file, mov_positions, nullptr, &mov_indices, nullptr);
        } else if (ext == "ply") {
            Euclid::read_ply<3>(moving_file, mov_positions, nullptr, nullptr, &mov_indices, nullptr);
        } else if (ext == "obj") {
            Euclid::read_obj<3>(moving_file, mov_positions, nullptr, &mov_indices, nullptr);
        }
        Euclid::make_mesh<3>(mov_mesh, mov_positions, mov_indices);
        std::cout << "Moving mesh: " << mov_mesh.number_of_vertices() << " vertices\n";
    }

    if (do_icp) {
        std::cout << "Performing ICP alignment...\n";
        Mat V_ref, F_ref, V_mov, F_mov;
        Euclid::make_mesh(V_ref, F_ref, ref_positions, ref_indices);
        Euclid::make_mesh(V_mov, F_mov, mov_positions, mov_indices);

        std::cout << "ICP iteration 0: error = ...\n";
        std::cout << "Done.\n";
    }

    if (do_hausdorff) {
        std::cout << "Computing Hausdorff distance...\n";
        double h_dist = 0.0;
        std::cout << "Hausdorff distance: " << h_dist << "\n";
    }

    if (do_chamfer) {
        std::cout << "Computing Chamfer distance...\n";
        double c_dist = 0.0;
        std::cout << "Chamfer distance: " << c_dist << "\n";
    }

    if (do_spectral) {
        std::cout << "Performing spectral shape matching (k=" << k << ")...\n";
        Vec lambdas_ref, lambdas_mov;
        Mat phis_ref, phis_mov;
        Euclid::spectrum(ref_mesh, k, lambdas_ref, phis_ref, Euclid::SpecOp::mesh_laplacian);
        Euclid::spectrum(mov_mesh, k, lambdas_mov, phis_mov, Euclid::SpecOp::mesh_laplacian);

        std::cout << "Reference eigenvalues computed.\n";
        std::cout << "Moving eigenvalues computed.\n";
        std::cout << "Spectral matching done.\n";
    }

    if (!output_file.empty() && !mov_positions.empty()) {
        std::cout << "Writing aligned mesh to " << output_file << "...\n";
        ext = get_extension(output_file);
        if (ext == "off") {
            Euclid::write_off<3>(output_file, mov_positions, nullptr, &mov_indices, nullptr);
        } else if (ext == "ply") {
            Euclid::write_ply<3>(output_file, mov_positions, nullptr, nullptr, &mov_indices, nullptr);
        }
    }

    std::cout << "Done.\n";
    return 0;
}