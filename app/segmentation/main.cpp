#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <unordered_map>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/ObjIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Segmentation/RandomWalk.h>
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
              << "Mesh segmentation tool using random walker algorithm\n\n"
              << "Options:\n"
              << "  -i <file>    Input mesh file (off, ply, obj)\n"
              << "  -o <file>    Output mesh file (colored by segment)\n"
              << "  -s <idx>     Seed vertex (can be specified multiple times)\n"
              << "  --random     Use random seed vertices\n"
              << "  --k <num>    Number of random seeds (default: 12)\n"
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

int main(int argc, char* argv[])
{
    std::string input_file;
    std::string output_file;
    std::vector<unsigned> seeds;
    bool random_seeds = false;
    int num_seeds = 12;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            seeds.push_back(std::atoi(argv[++i]));
        } else if (strcmp(argv[i], "--random") == 0) {
            random_seeds = true;
        } else if (strcmp(argv[i], "--k") == 0 && i + 1 < argc) {
            num_seeds = std::atoi(argv[++i]);
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
    std::vector<unsigned> indices;

    std::cout << "Reading mesh from " << input_file << "...\n";
    read_mesh(input_file, positions, nullptr, indices);

    Mesh mesh;
    Euclid::make_mesh<3>(mesh, positions, indices);
    std::cout << "Loaded " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces.\n";

    if (random_seeds && seeds.empty()) {
        std::cout << "Generating " << num_seeds << " random seed vertices...\n";
        seeds.resize(num_seeds);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, mesh.number_of_vertices() - 1);

        for (int i = 0; i < num_seeds; ++i) {
            seeds[i] = dis(gen);
        }
    }

    if (seeds.empty()) {
        std::cerr << "Error: No seed vertices specified. Use -s or --random\n\n";
        print_usage(argv[0]);
        return 1;
    }

    std::cout << "Performing random walker segmentation with " << seeds.size()
              << " seeds...\n";
    for (size_t i = 0; i < seeds.size(); ++i) {
        std::cout << "  Seed " << i << ": vertex " << seeds[i] << "\n";
    }

    std::vector<unsigned> segments;
    Euclid::random_walk_segmentation(mesh, seeds, segments);

    std::unordered_map<unsigned, unsigned> sid;
    for (size_t i = 0; i < seeds.size(); ++i) {
        sid.emplace(seeds[i], i);
    }

    std::vector<unsigned char> color_set;
    Euclid::rnd_colors(seeds.size(), color_set, true);

    std::vector<unsigned char> colors(segments.size() * 4);
    for (size_t i = 0; i < segments.size(); ++i) {
        auto id = sid[segments[i]];
        colors[4 * i + 0] = color_set[3 * id + 0];
        colors[4 * i + 1] = color_set[3 * id + 1];
        colors[4 * i + 2] = color_set[3 * id + 2];
        colors[4 * i + 3] = 255;
    }

    if (!output_file.empty()) {
        std::cout << "Writing segmented mesh to " << output_file << "...\n";
        Euclid::write_ply<3>(output_file, positions, nullptr, &indices, &colors);
    }

    std::unordered_map<unsigned, int> segment_counts;
    for (const auto& seg : segments) {
        segment_counts[seg]++;
    }

    std::cout << "Segmentation complete. Segments:\n";
    for (const auto& [seed, count] : segment_counts) {
        std::cout << "  Segment " << seed << ": " << count << " vertices\n";
    }

    std::cout << "Done.\n";
    return 0;
}