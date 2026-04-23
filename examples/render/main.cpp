#include <string>
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/BoundingVolume/AABB.h>
#include <Euclid/Render/RayTracer.h>
#include <Euclid/Math/Vector.h>

#include <config.h>
#include <stb_image_write.h>

using Kernel = CGAL::Simple_cartesian<float>;

static std::string get_extension(const std::string& path)
{
    auto pos = path.rfind('.');
    if (pos != std::string::npos) {
        return path.substr(pos + 1);
    }
    return "";
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
    } else if (ext == "stl") {
        Euclid::read_stl<3>(path, positions, normals, &indices, nullptr);
    }
}

int main(int argc, char* argv[])
{
    const char* input = "bunny.off";
    const char* output = "render.png";
    int width = 800;
    int height = 600;
    float fov = 60.0f;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-i" && i + 1 < argc) {
            input = argv[++i];
        } else if (std::string(argv[i]) == "-o" && i + 1 < argc) {
            output = argv[++i];
        } else if (std::string(argv[i]) == "--width" && i + 1 < argc) {
            width = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "--height" && i + 1 < argc) {
            height = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "--fov" && i + 1 < argc) {
            fov = std::atof(argv[++i]);
        }
    }

    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<unsigned> indices;
    std::string fmesh(DATA_DIR);
    fmesh.append(input);
    read_mesh(fmesh, positions, &normals, indices);

    Euclid::AABB<Kernel> aabb(positions);
    Eigen::Vector3f center;
    Euclid::cgal_to_eigen(aabb.center(), center);
    Eigen::Vector3f view =
        center + Eigen::Vector3f(0.0f, 0.5f * aabb.ylen(), 2.0f * aabb.zlen());
    Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

    Euclid::RayTracer raytracer;
    positions.push_back(0.0f);
    raytracer.attach_geometry_buffers(positions, indices);

    const float near = 0.001f;
    const float far = 10.0f;

    Euclid::PerspRayCamera cam(
        view, center, up, fov, width, height, near, far);

    std::vector<uint8_t> pixels(3 * width * height);
    raytracer.render_shaded(pixels, cam, width, height);

    std::string fout(TMP_DIR);
    fout.append(output);
    stbi_write_png(
        fout.c_str(), width, height, 3, pixels.data(), width * 3);
}