#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/Descriptor/SpinImage.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Util/Color.h>

#include <config.h>
#include <stb_image_write.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;

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

static void write_spin_image(const std::string& f,
                        const Eigen::ArrayXd& si,
                        int width)
{
    auto vmax = si.maxCoeff();
    Eigen::ArrayXd sig = si;
    sig *= 255.0 / vmax;
    std::string fout(TMP_DIR);
    fout.append(f);
    stbi_write_png(
        fout.c_str(), width, width, 1, &sig(0), width * sizeof(double));
}

int main(int argc, char* argv[])
{
    const char* input = "dragon.ply";
    const char* output = "spinimage_distances.ply";
    int vertex = 21785;
    unsigned width = 16;
    float angle = 90.0f;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-i" && i + 1 < argc) {
            input = argv[++i];
        } else if (std::string(argv[i]) == "-o" && i + 1 < argc) {
            output = argv[++i];
        } else if (std::string(argv[i]) == "-v" && i + 1 < argc) {
            vertex = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "--width" && i + 1 < argc) {
            width = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "--angle" && i + 1 < argc) {
            angle = std::atof(argv[++i]);
        }
    }

    std::vector<double> positions;
    std::vector<double> normals;
    std::vector<unsigned> indices;
    std::string fmesh(DATA_DIR);
    fmesh.append(input);
    read_mesh(fmesh, positions, &normals, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);

    Euclid::SpinImage<Mesh> si;
    si.build(mesh);

    Eigen::ArrayXXd si_all;
    si.compute(si_all, 1.0f, width, angle);

    write_spin_image("spinimage.png", si_all.col(vertex), width);

    std::vector<double> distances(si_all.cols());
    for (int i = 0; i < si_all.cols(); ++i) {
        distances[i] = Euclid::chi2(si_all.col(i), si_all.col(vertex));
    }
    std::vector<unsigned char> colors;
    Euclid::colormap(
        igl::COLOR_MAP_TYPE_JET, distances, colors, true, false, true);
    std::string fout(TMP_DIR);
    fout.append(output);
    Euclid::write_ply<3>(
        fout, positions, nullptr, nullptr, &indices, &colors);
}