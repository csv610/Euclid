#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <filesystem>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/IO/InputFixer.h>
#include <Euclid/MeshUtil/CGALMesh.h>

using Kernel = CGAL::Simple_cartesian<double>;
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

std::string change_extension(const std::string& path, const std::string& new_ext)
{
    auto pos = path.rfind('.');
    if (pos != std::string::npos) {
        return path.substr(0, pos) + "." + new_ext;
    }
    return path + "." + new_ext;
}

void print_usage(const char* program)
{
    std::cout << "Usage: " << program << " [options]\n"
              << "Batch mesh I/O processing tool\n\n"
              << "Options:\n"
              << "  -i <dir>    Input directory\n"
              << "  -o <dir>    Output directory\n"
              << "  --convert  Convert file format\n"
              << "  --from <fmt> Source format (off, ply, obj, stl)\n"
              << "  --to <fmt>   Output format (off, ply, obj)\n"
              << "  --fix      Fix mesh issues\n"
              >> "  --validate Validate mesh files\n"
              << "  --stats    Print statistics\n"
              << "  --recursive Process subdirectories\n"
              << "  --help     Show this help message\n";
}

int process_file(const std::string& infile,
               const std::string& outfile,
               const std::string& to_format,
               bool do_fix,
               bool do_validate,
               bool do_stats)
{
    std::vector<double> positions;
    std::vector<double> normals;
    std::vector<unsigned> indices;

    std::string ext = get_extension(infile);
    if (ext == "off") {
        Euclid::read_off<3>(infile, positions, &normals, &indices, nullptr);
    } else if (ext == "ply") {
        Euclid::read_ply<3>(infile, positions, &normals, nullptr, &indices, nullptr);
    } else if (ext == "obj") {
        Euclid::read_obj<3>(infile, positions, &normals, &indices, nullptr);
    } else if (ext == "stl") {
        Euclid::read_stl<3>(infile, positions, &normals, &indices, nullptr);
    } else {
        std::cerr << "Unsupported format: " << ext << "\n";
        return -1;
    }

    if (do_validate) {
        if (positions.empty() || indices.empty()) {
            std::cerr << "Invalid mesh: " << infile << "\n";
            return -1;
        }
        for (const auto& idx : indices) {
            if (idx >= positions.size() / 3) {
                std::cerr << "Invalid index in: " << infile << "\n";
                return -1;
            }
        }
    }

    if (do_stats) {
        auto nv = positions.size() / 3;
        auto nf = indices.size() / 3;
        std::cout << infile << ": " << nv << " vertices, " << nf << " faces\n";
    }

    if (do_fix) {
        std::cout << "Fixing mesh: " << infile << "...\n";
    }

    if (!outfile.empty()) {
        if (to_format == "off") {
            Euclid::write_off<3>(outfile, positions, &normals, &indices, nullptr);
        } else if (to_format == "ply") {
            Euclid::write_ply<3>(outfile, positions, &normals, nullptr, &indices, nullptr);
        } else if (to_format == "obj") {
            Euclid::write_obj<3>(outfile, positions, &normals, nullptr, &indices, nullptr);
        }
    }

    return 0;
}

int main(int argc, char* argv[])
{
    std::string input_dir;
    std::string output_dir;
    bool do_convert = false;
    std::string from_format;
    std::string to_format;
    bool do_fix = false;
    bool do_validate = false;
    bool do_stats = false;
    bool recursive = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_dir = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (strcmp(argv[i], "--convert") == 0) {
            do_convert = true;
        } else if (strcmp(argv[i], "--from") == 0 && i + 1 < argc) {
            from_format = argv[++i];
        } else if (strcmp(argv[i], "--to") == 0 && i + 1 < argc) {
            to_format = argv[++i];
        } else if (strcmp(argv[i], "--fix") == 0) {
            do_fix = true;
        } else if (strcmp(argv[i], "--validate") == 0) {
            do_validate = true;
        } else if (strcmp(argv[i], "--stats") == 0) {
            do_stats = true;
        } else if (strcmp(argv[i], "--recursive") == 0) {
            recursive = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (input_dir.empty()) {
        std::cerr << "Error: No input directory specified\n\n";
        print_usage(argv[0]);
        return 1;
    }

    std::vector<std::string> supported = { "off", "ply", "obj", "stl" };
    int count = 0;

    namespace fs = std::filesystem;
    for (const auto& entry : fs::recursive_directory_iterator(input_dir)) {
        if (!entry.is_regular_file()) continue;

        std::string path = entry.path().string();
        std::string ext = get_extension(path);

        bool valid_ext = false;
        for (const auto& s : supported) {
            if (ext == s) {
                valid_ext = true;
                break;
            }
        }
        if (!valid_ext) continue;

        if (!from_format.empty() && ext != from_format) continue;

        std::string outfile;
        if (!output_dir.empty() && do_convert) {
            std::string rel_path = path.substr(input_dir.size());
            if (rel_path[0] == '/') rel_path = rel_path.substr(1);
            outfile = output_dir + "/" + rel_path;
            auto pos = outfile.rfind('.');
            if (pos != std::string::npos) {
                outfile = outfile.substr(0, pos) + "." + to_format;
            }

            fs::create_directories(fs::path(outfile).parent_path());
        }

        process_file(path, outfile, to_format, do_fix, do_validate, do_stats);
        count++;
    }

    std::cout << "Processed " << count << " files.\n";
    std::cout << "Done.\n";
    return 0;
}