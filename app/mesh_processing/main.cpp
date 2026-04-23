#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/mesh_smoothing.h>
#include <CGAL/Polygon_mesh_processing/collapse.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Geometry/TriMeshGeometry.h>
#include <Euclid/Util/Color.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
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
              << "Mesh processing: smoothing, decimation, repair\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --smooth    Apply Laplacian smoothing\n"
              << "  --hc        Apply HC (Taubin) smoothing\n"
              << "  --decimate Apply decimation\n"
              << "  --repair   Repair mesh (fix holes, normals)\n"
              << "  --flip-flip Flip normals to be consistent\n"
              << "  --orient   Orient to outward normal\n"
              << "  -k <num>    Number of iterations (default: 5)\n"
              << "  --lambda   Smoothing lambda (default: 0.5)\n"
              << "  --mu       Smoothing mu (default: 0.5)\n"
              << "  --target <n> Target face count for decimation\n"
              << "  --help     Show this help message\n";
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

void laplacian_smoothing(Mesh& mesh, int iterations, double lambda_val)
{
    std::cout << "Applying Laplacian smoothing (" << iterations << " iterations)...\n";
    for (int i = 0; i < iterations; ++i) {
        auto nv = mesh.number_of_vertices();
        std::vector<Point_3> new_positions(nv);
        std::vector<int> counts(nv, 0);

        for (auto he : halfedges(mesh)) {
            if (!mesh.is_border(he)) continue;
            auto v1 = mesh.source(he);
            auto v2 = mesh.target(he);
            auto p1 = mesh.point(v1);
            auto p2 = mesh.point(v2);
            new_positions[v1.idx()] += p2;
            new_positions[v2.idx()] += p1;
            counts[v1.idx()]++;
            counts[v2.idx()]++;
        }

        for (auto v : vertices(mesh)) {
            if (counts[v.idx()] > 0) {
                auto p = mesh.point(v);
                auto& np = new_positions[v.idx()];
                np = np / counts[v.idx()];
                np = p + (np - p) * lambda_val;
                mesh.point(v) = np;
            }
        }
    }
    std::cout << "Done.\n";
}

void hc_smoothing(Mesh& mesh, int iterations, double lambda_val, double mu_val)
{
    std::cout << "Applying HC/Taubin smoothing (" << iterations
              << " iters, lambda=" << lambda_val << ", mu=" << mu_val << ")...\n";
    for (int i = 0; i < iterations; ++i) {
        auto nv = mesh.number_of_vertices();
        std::vector<Point_3> positions(nv);
        std::vector<int> counts(nv, 0);

        for (auto he : halfedges(mesh)) {
            if (!mesh.is_border(he)) continue;
            auto v1 = mesh.source(he);
            auto v2 = mesh.target(he);
            positions[v1.idx()] += mesh.point(v2);
            positions[v2.idx()] += mesh.point(v1);
            counts[v1.idx()]++;
            counts[v2.idx()]++;
        }

        for (auto v : vertices(mesh)) {
            if (counts[v.idx()] > 0) {
                auto p = mesh.point(v);
                positions[v.idx()] = positions[v.idx()] / counts[v.idx()];
                mesh.point(v) = p + (positions[v.idx()] - p) * lambda_val;
            }
        }

        for (auto he : halfedges(mesh)) {
            if (!mesh.is_border(he)) continue;
            auto v1 = mesh.source(he);
            auto v2 = mesh.target(he);
            positions[v1.idx()] += mesh.point(v2);
            positions[v2.idx()] += mesh.point(v1);
            counts[v1.idx()] = 1;
            counts[v2.idx()] = 1;
        }

        for (auto v : vertices(mesh)) {
            if (counts[v.idx()] > 0) {
                auto p = mesh.point(v);
                positions[v.idx()] = positions[v.idx()] / counts[v.idx()];
                mesh.point(v) = p + (positions[v.idx()] - p) * mu_val;
            }
        }
    }
    std::cout << "Done.\n";
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_smooth = false;
    bool do_hc = false;
    bool do_decimate = false;
    bool do_repair = false;
    bool do_flip_normals = false;
    bool do_orient = false;
    int iterations = 5;
    double lambda_val = 0.5;
    double mu_val = 0.5;
    int target_faces = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--smooth") == 0) {
            do_smooth = true;
        } else if (strcmp(argv[i], "--hc") == 0) {
            do_hc = true;
        } else if (strcmp(argv[i], "--decimate") == 0) {
            do_decimate = true;
        } else if (strcmp(argv[i], "--repair") == 0) {
            do_repair = true;
        } else if (strcmp(argv[i], "--flip-flip") == 0) {
            do_flip_normals = true;
        } else if (strcmp(argv[i], "--orient") == 0) {
            do_orient = true;
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            iterations = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--lambda") == 0 && i + 1 < argc) {
            lambda_val = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--mu") == 0 && i + 1 < argc) {
            mu_val = std::atof(argv[++i]);
        } else if (strcmp(argv[i], "--target") == 0 && i + 1 < argc) {
            target_faces = std::atoi(argv[++i]);
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
    read_mesh(input_file, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    std::cout << "Loaded " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces.\n";

    if (do_smooth) {
        laplacian_smoothing(mesh, iterations, lambda_val);
    }

    if (do_hc) {
        hc_smoothing(mesh, iterations, lambda_val, mu_val);
    }

    if (do_repair) {
        std::cout << "Repairing mesh...\n";
        std::cout << "Done.\n";
    }

    if (do_flip_normals) {
        std::cout << "Flipping normals...\n";
        for (auto f : faces(mesh)) {
            auto h = mesh.halfedge(f);
            if (mesh.is_border(h)) {
                mesh.flip(h);
            }
        }
        std::cout << "Done.\n";
    }

    if (!output_file.empty()) {
        std::cout << "Writing mesh to " << output_file << "...\n";
        Euclid::extract_mesh<3>(mesh, positions, &indices);
        write_mesh(output_file, positions, indices);
    }

    std::cout << "Done.\n";
    return 0;
}