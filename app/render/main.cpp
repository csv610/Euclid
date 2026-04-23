#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <cstring>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Eigen/Core>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Render/RenderCore.h>
#include <Euclid/Geometry/TriMeshGeometry.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Vec3 = Eigen::Vector3f;

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
              << "Mesh rendering and camera setup tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output image file (png, ppm)\n"
              << "  --camera    Setup camera position\n"
              << "  --front    Set camera to front view\n"
              << "  --back     Set camera to back view\n"
              << "  --left    Set camera to left view\n"
              << "  --right   Set camera to right view\n"
              << "  --top     Set camera to top view\n"
              << "  --bottom  Set camera to bottom view\n"
              << "  --iso     Set camera to isometric view\n"
              << "  --center <x,y,z> Camera look at center\n"
              << "  --pos <x,y,z> Camera position\n"
              << "  --wire    Render wireframe\n"
              << "  --flat    Render flat shading\n"
              << "  --smooth  Render smooth shading\n"
              << "  -w <num>   Image width (default: 800)\n"
              << "  -h <num>   Image height (default: 600)\n"
              << "  --help    Show this help message\n";
}

Eigen::Vector3f parse_vector3(const std::string& str)
{
    Eigen::Vector3f v;
    std::string s = str;
    std::replace(s.begin(), s.end(), ',', ' ');
    std::stringstream ss(s);
    ss >> v[0] >> v[1] >> v[2];
    return v;
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    bool do_camera = false;
    bool do_front = false;
    bool do_back = false;
    bool do_left = false;
    bool do_right = false;
    bool do_top = false;
    bool do_bottom = false;
    bool do_iso = false;
    Eigen::Vector3f center{0, 0, 0};
    Eigen::Vector3f cam_pos{0, 0, 1};
    bool do_wire = false;
    bool do_flat = false;
    bool do_smooth = true;
    int width = 800;
    int height = 600;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--camera") == 0) {
            do_camera = true;
        } else if (strcmp(argv[i], "--front") == 0) {
            do_front = true;
        } else if (strcmp(argv[i], "--back") == 0) {
            do_back = true;
        } else if (strcmp(argv[i], "--left") == 0) {
            do_left = true;
        } else if (strcmp(argv[i], "--right") == 0) {
            do_right = true;
        } else if (strcmp(argv[i], "--top") == 0) {
            do_top = true;
        } else if (strcmp(argv[i], "--bottom") == 0) {
            do_bottom = true;
        } else if (strcmp(argv[i], "--iso") == 0) {
            do_iso = true;
        } else if (strcmp(argv[i], "--center") == 0 && i + 1 < argc) {
            center = parse_vector3(argv[++i]);
        } else if (strcmp(argv[i], "--pos") == 0 && i + 1 < argc) {
            cam_pos = parse_vector3(argv[++i]);
        } else if (strcmp(argv[i], "--wire") == 0) {
            do_wire = true;
        } else if (strcmp(argv[i], "--flat") == 0) {
            do_flat = true;
            do_smooth = false;
        } else if (strcmp(argv[i], "--smooth") == 0) {
            do_smooth = true;
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            width = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            height = std::atoi(argv[++i]);
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

    Euclid::Camera camera;
    if (do_front) {
        cam_pos = {0, 0, 1};
    } else if (do_back) {
        cam_pos = {0, 0, -1};
    } else if (do_left) {
        cam_pos = {-1, 0, 0};
    } else if (do_right) {
        cam_pos = {1, 0, 0};
    } else if (do_top) {
        cam_pos = {0, 1, 0};
    } else if (do_bottom) {
        cam_pos = {0, -1, 0};
    } else if (do_iso) {
        cam_pos = {1, 1, 1};
    }

    if (do_camera || do_front || do_back || do_left || do_right || do_top || do_bottom || do_iso) {
        camera.lookat(cam_pos, center, Vec3{0, 1, 0});
        camera.set_range(0.01f, 100.0f);
        std::cout << "Camera setup:\n";
        std::cout << "  Position: (" << camera.pos[0] << ", " << camera.pos[1] << ", " << camera.pos[2] << ")\n";
        std::cout << "  Looking at: (" << center[0] << ", " << center[1] << ", " << center[2] << ")\n";
    }

    Euclid::Material material;
    material.ambient = Eigen::Array3f(0.1f, 0.1f, 0.1f);
    material.diffuse = Eigen::Array3f(0.8f, 0.8f, 0.8f);

    Euclid::Light light;
    light.position = Eigen::Array3f(1.0f, 1.0f, 1.0f);
    light.color = Eigen::Array3f(1.0f, 1.0f, 1.0f);
    light.intensity = 1.0f;

    std::cout << "Rendering settings:\n";
    std::cout << "  Resolution: " << width << "x" << height << "\n";
    std::cout << "  Shading: " << (do_wire ? "wireframe" : (do_flat ? "flat" : "smooth")) << "\n";
    std::cout << "  Material: diffuse=(" << material.diffuse[0] << "," << material.diffuse[1] << "," << material.diffuse[2] << ")\n";

    if (!output_file.empty()) {
        std::cout << "Rendering to " << output_file << "...\n";
        std::cout << "(Note: Full rendering requires Embree. This shows camera setup only.)\n";
    }

    std::cout << "Done.\n";
    return 0;
}