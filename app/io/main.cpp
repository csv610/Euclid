#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/IO/InputFixer.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Geometry/TriMeshGeometry.h>
#include <Euclid/Util/Color.h>

using Kernel = CGAL::Simple_cartesian<float>;
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
              << "Mesh I/O conversion tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file\n"
              << "  --color      Compute and apply curvature-based coloring\n"
              << "  --normalize  Normalize mesh to unit cube\n"
              << "  --center     Center mesh at origin\n"
              << "  --info       Print mesh information only\n"
              << "  --help       Show this help message\n";
}

template<typename T>
void read_mesh(const std::string& path,
               std::vector<T>& positions,
               std::vector<T>* normals,
               std::vector<unsigned>& indices,
               std::vector<unsigned char>* colors)
{
    std::string ext = get_extension(path);
    if (ext == "off") {
        Euclid::read_off<3>(path, positions, colors, &indices, nullptr);
    } else if (ext == "ply") {
        Euclid::read_ply<3>(path, positions, normals, (std::vector<T>*)nullptr, &indices, colors);
    } else if (ext == "obj") {
        Euclid::read_obj<3>(path, positions, indices, (std::vector<T>*)nullptr, (std::vector<unsigned>*)nullptr, normals, (std::vector<unsigned>*)nullptr);
    } else {
        std::cerr << "Unsupported format: " << ext << std::endl;
    }
}

template<typename T>
void write_mesh(const std::string& path,
                const std::vector<T>& positions,
                const std::vector<T>* normals,
                const std::vector<unsigned>& indices,
                const std::vector<unsigned char>* colors,
                const std::vector<T>* texcoords)
{
    std::string ext = get_extension(path);
    if (ext == "off") {
        Euclid::write_off<3>(path, positions, colors, &indices, nullptr);
    } else if (ext == "ply") {
        Euclid::write_ply<3>(path, positions, normals, texcoords, &indices, colors);
    } else if (ext == "obj") {
        Euclid::write_obj<3>(path, positions, indices, texcoords, (std::vector<unsigned>*)nullptr, normals, (std::vector<unsigned>*)nullptr);
    } else {
        std::cerr << "Unsupported output format: " << ext << std::endl;
    }
}

void print_mesh_info(const std::vector<float>& positions,
                     const std::vector<unsigned>& indices)
{
    auto nv = positions.size() / 3;
    auto nf = indices.size() / 3;

    if (nv == 0) return;

    float min_x = positions[0], max_x = positions[0];
    float min_y = positions[1], max_y = positions[1];
    float min_z = positions[2], max_z = positions[2];

    for (size_t i = 0; i < nv; ++i) {
        min_x = std::min(min_x, positions[i * 3 + 0]);
        max_x = std::max(max_x, positions[i * 3 + 0]);
        min_y = std::min(min_y, positions[i * 3 + 1]);
        max_y = std::max(max_y, positions[i * 3 + 1]);
        min_z = std::min(min_z, positions[i * 3 + 2]);
        max_z = std::max(max_z, positions[i * 3 + 2]);
    }

    std::cout << "Mesh Information:\n"
              << "  Vertices: " << nv << "\n"
              << "  Faces: " << nf << "\n"
              << "  Bounding Box:\n"
              << "    X: [" << min_x << ", " << max_x << "]\n"
              << "    Y: [" << min_y << ", " << max_y << "]\n"
              << "    Z: [" << min_z << ", " << max_z << "]\n";
}

void center_mesh(std::vector<float>& positions)
{
    auto nv = positions.size() / 3;
    if (nv == 0) return;
    float cx = 0, cy = 0, cz = 0;

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
}

void normalize_mesh(std::vector<float>& positions)
{
    auto nv = positions.size() / 3;
    if (nv == 0) return;
    float min_x = positions[0], max_x = positions[0];
    float min_y = positions[1], max_y = positions[1];
    float min_z = positions[2], max_z = positions[2];

    for (size_t i = 0; i < nv; ++i) {
        min_x = std::min(min_x, positions[i * 3 + 0]);
        max_x = std::max(max_x, positions[i * 3 + 0]);
        min_y = std::min(min_y, positions[i * 3 + 1]);
        max_y = std::max(max_y, positions[i * 3 + 1]);
        min_z = std::min(min_z, positions[i * 3 + 2]);
        max_z = std::max(max_z, positions[i * 3 + 2]);
    }

    float scale = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
    if (scale > 0) {
        float cx = (min_x + max_x) / 2;
        float cy = (min_y + max_y) / 2;
        float cz = (min_z + max_z) / 2;

        for (size_t i = 0; i < nv; ++i) {
            positions[i * 3 + 0] = (positions[i * 3 + 0] - cx) / scale;
            positions[i * 3 + 1] = (positions[i * 3 + 1] - cy) / scale;
            positions[i * 3 + 2] = (positions[i * 3 + 2] - cz) / scale;
        }
    }
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_color = false;
    bool do_normalize = false;
    bool do_center = false;
    bool do_info = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--color") == 0) {
            do_color = true;
        } else if (strcmp(argv[i], "--normalize") == 0) {
            do_normalize = true;
        } else if (strcmp(argv[i], "--center") == 0) {
            do_center = true;
        } else if (strcmp(argv[i], "--info") == 0) {
            do_info = true;
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

    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<unsigned> indices;
    std::vector<unsigned char> colors;

    std::cout << "Reading mesh from " << input_file << "...\n";
    read_mesh(input_file, positions, &normals, indices, &colors);
    std::cout << "Done. Loaded " << (positions.size() / 3) << " vertices, "
              << (indices.size() / 3) << " faces.\n";

    if (do_info) {
        print_mesh_info(positions, indices);
        return 0;
    }

    if (do_center) {
        std::cout << "Centering mesh...\n";
        center_mesh(positions);
    }

    if (do_normalize) {
        std::cout << "Normalizing mesh to unit cube...\n";
        normalize_mesh(positions);
    }

    if (do_color) {
        std::cout << "Computing curvature-based coloring...\n";
        Mesh mesh;
        Euclid::make_mesh<3>(mesh, positions, indices);
        auto curvatures = Euclid::gaussian_curvatures(mesh);
        Euclid::colormap(igl::COLOR_MAP_TYPE_JET, curvatures, colors, true);
    }

    if (output_file.empty()) {
        if (!do_info) {
             std::cerr << "Error: No output file specified\n\n";
             print_usage(argv[0]);
             return 1;
        }
        return 0;
    }

    std::cout << "Writing mesh to " << output_file << "...\n";
    std::vector<float> texcoords;
    write_mesh<float>(output_file, positions, &normals, indices,
                do_color ? &colors : (const std::vector<unsigned char>*)nullptr, 
                (const std::vector<float>*)nullptr);
    std::cout << "Done.\n";

    return 0;
}
