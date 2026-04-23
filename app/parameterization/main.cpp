#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <fstream>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Parameterization/SCP.h>
#include <Euclid/Parameterization/RicciFlow.h>
#include <Euclid/Parameterization/HolomorphicOneForms.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Vertex = boost::graph_traits<Mesh>::vertex_descriptor;
using UV_Map = std::unordered_map<Vertex, Point_2>;
using UV_PMap = boost::associative_property_map<UV_Map>;

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
              << "Mesh parameterization and UV mapping tool\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file (with UV coords)\n"
              << "  --scp        Spectral conformal parameterization\n"
              << "  --ricci      Ricci flow conformal parameterization\n"
              << "  --holomorphic Holomorphic one-forms parameterization\n"
              << "  -k <num>     Number of Ricci flow iterations (default: 100)\n"
              << "  --disk       Output UV domain as .off file\n"
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

void uv_to_texcoords(const Mesh& mesh, const UV_Map& uvm, std::vector<double>& texcoords)
{
    texcoords.resize(mesh.number_of_vertices() * 2);
    for (auto v : vertices(mesh)) {
        auto uv = uvm.at(v);
        texcoords[v.idx() * 2 + 0] = uv.x();
        texcoords[v.idx() * 2 + 1] = uv.y();
    }
}

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    std::string disk_file;
    bool do_scp = false;
    bool do_ricci = false;
    bool do_holomorphic = false;
    int iterations = 100;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "--scp") == 0) {
            do_scp = true;
        } else if (strcmp(argv[i], "--ricci") == 0) {
            do_ricci = true;
        } else if (strcmp(argv[i], "--holomorphic") == 0) {
            do_holomorphic = true;
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            iterations = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "--disk") == 0 && i + 1 < argc) {
            disk_file = argv[++i];
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (!do_scp && !do_ricci && !do_holomorphic) {
        do_scp = true;
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

    std::vector<double> texcoords;

    if (do_scp) {
        std::cout << "Computing Spectral Conformal Parameterization...\n";

        auto bnd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;
        UV_Map uvm;
        UV_PMap uvpm(uvm);

        Euclid::SCP_parameterizer_3<Mesh> param;
        CGAL::Surface_mesh_parameterization::parameterize(mesh, param, bnd, uvpm);

        uv_to_texcoords(mesh, uvm, texcoords);
        std::cout << "SCP parameterization done.\n";

        if (!disk_file.empty()) {
            std::ofstream ofs(disk_file);
            CGAL::Surface_mesh_parameterization::IO::output_uvmap_to_off(
                mesh, bnd, uvpm, ofs);
            std::cout << "UV domain written to " << disk_file << "\n";
        }
    }

    if (do_ricci) {
        std::cout << "Computing Ricci Flow parameterization (iterations=" << iterations << ")...\n";

        UV_Map uvm;
        UV_PMap uvpm(uvm);

        Euclid::RicciFlowParameterizer<Mesh> param;
        param.set_iterations(iterations);
        auto bnd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;
        CGAL::Surface_mesh_parameterization::parameterize(mesh, param, bnd, uvpm);

        uv_to_texcoords(mesh, uvm, texcoords);
        std::cout << "Ricci flow parameterization done.\n";

        if (!disk_file.empty()) {
            std::ofstream ofs(disk_file);
            CGAL::Surface_mesh_parameterization::IO::output_uvmap_to_off(
                mesh, bnd, uvpm, ofs);
            std::cout << "UV domain written to " << disk_file << "\n";
        }
    }

    if (do_holomorphic) {
        std::cout << "Computing Holomorphic One-Forms parameterization...\n";

        UV_Map uvm;
        UV_PMap uvpm(uvm);

        Euclid::HolomorphicOneFormsParameterizer<Mesh> param;
        auto bnd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;
        CGAL::Surface_mesh_parameterization::parameterize(mesh, param, bnd, uvpm);

        uv_to_texcoords(mesh, uvm, texcoords);
        std::cout << "Holomorphic one-forms parameterization done.\n";

        if (!disk_file.empty()) {
            std::ofstream ofs(disk_file);
            CGAL::Surface_mesh_parameterization::IO::output_uvmap_to_off(
                mesh, bnd, uvpm, ofs);
            std::cout << "UV domain written to " << disk_file << "\n";
        }
    }

    if (!output_file.empty() && !texcoords.empty()) {
        std::cout << "Writing mesh with UV coordinates to " << output_file << "...\n";
        Euclid::write_ply<3>(output_file, positions, nullptr, &texcoords, &indices, nullptr);
    }

    std::cout << "Done.\n";
    return 0;
}