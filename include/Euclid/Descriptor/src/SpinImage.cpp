#include <cmath>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <limits>

#include <boost/math/constants/constants.hpp>
#include <Euclid/Geometry/TriMeshGeometry.h>
#include <Euclid/Math/Vector.h>

namespace Euclid
{

template<typename Mesh>
void SpinImage<Mesh>::build(const Mesh& mesh,
                            const std::vector<Vector_3>* vnormals,
                            FT resolution)
{
    this->mesh = &mesh;

    if (vnormals != nullptr) {
        this->vnormals.reset(vnormals, false);
    }
    else {
        auto face_normals = Euclid::face_normals(mesh);
        auto vert_normals = Euclid::vertex_normals(mesh, face_normals);
        this->vnormals.reset(new std::vector<Vector_3>(vert_normals), true);
    }

    if (resolution != 0.0) {
        this->resolution = resolution;
    }
    else {
        this->resolution = 0.0;
        for (auto e : edges(mesh)) {
            this->resolution += edge_length(e, mesh);
        }
        this->resolution /= static_cast<FT>(num_edges(mesh));
    }
}

template<typename Mesh>
template<typename Derived>
void SpinImage<Mesh>::compute(Eigen::ArrayBase<Derived>& spin_img,
                              float bin_scale,
                              int image_width,
                              float support_angle)
{
    auto vpmap = get(boost::vertex_point, *this->mesh);
    auto vimap = get(boost::vertex_index, *this->mesh);
    auto cos_range =
        std::cos(support_angle * boost::math::float_constants::degree);
    auto bin_size = this->resolution * static_cast<FT>(bin_scale);
    auto support_distance = bin_size * image_width;
    auto beta_max = support_distance * 0.5;
    spin_img.derived().setZero(image_width * image_width,
                               num_vertices(*this->mesh));

    // Build spatial grid
    FT xmin = std::numeric_limits<FT>::max(), xmax = -xmin;
    FT ymin = xmin, ymax = xmax;
    FT zmin = xmin, zmax = xmax;
    for (auto v : vertices(*this->mesh)) {
        auto p = get(vpmap, v);
        if (p.x() < xmin) xmin = p.x(); if (p.x() > xmax) xmax = p.x();
        if (p.y() < ymin) ymin = p.y(); if (p.y() > ymax) ymax = p.y();
        if (p.z() < zmin) zmin = p.z(); if (p.z() > zmax) zmax = p.z();
    }
    auto grid_size = static_cast<FT>(support_distance * 1.25);
    auto inv_grid_size = static_cast<FT>(1.0) / grid_size;
    int nx = static_cast<int>(std::ceil((xmax - xmin) * inv_grid_size)) + 1;
    int ny = static_cast<int>(std::ceil((ymax - ymin) * inv_grid_size)) + 1;
    int nz = static_cast<int>(std::ceil((zmax - zmin) * inv_grid_size)) + 1;
    std::vector<std::vector<typename boost::graph_traits<Mesh>::vertex_descriptor>> grid(nx * ny * nz);
    for (auto v : vertices(*this->mesh)) {
        auto p = get(vpmap, v);
        int gx = static_cast<int>((p.x() - xmin) * inv_grid_size);
        int gy = static_cast<int>((p.y() - ymin) * inv_grid_size);
        int gz = static_cast<int>((p.z() - zmin) * inv_grid_size);
        grid[gx * ny * nz + gy * nz + gz].push_back(v);
    }

    for (auto vi : vertices(*this->mesh)) {
        auto ii = get(vimap, vi);
        auto pi = get(vpmap, vi);
        auto ni = (*this->vnormals)[ii];

        int gx = static_cast<int>((pi.x() - xmin) * inv_grid_size);
        int gy = static_cast<int>((pi.y() - ymin) * inv_grid_size);
        int gz = static_cast<int>((pi.z() - zmin) * inv_grid_size);

        for (int dx = -1; dx <= 1; ++dx) {
            if (gx + dx < 0 || gx + dx >= nx) continue;
            for (int dy = -1; dy <= 1; ++dy) {
                if (gy + dy < 0 || gy + dy >= ny) continue;
                for (int dz = -1; dz <= 1; ++dz) {
                    if (gz + dz < 0 || gz + dz >= nz) continue;
                    
                    const auto& cell = grid[(gx + dx) * ny * nz + (gy + dy) * nz + (gz + dz)];
                    for (auto vj : cell) {
                        auto ij = get(vimap, vj);
                        auto pj = get(vpmap, vj);

                        if (ni * (*this->vnormals)[ij] < cos_range) {
                            continue;
                        }

                        auto beta = ni * (pj - pi);
                        auto alpha = std::sqrt((pj - pi).squared_length() - beta * beta);

                        auto col = static_cast<int>(std::floor(alpha / bin_size));
                        if (col > image_width - 2) {
                            continue;
                        }
                        auto row =
                            static_cast<int>(std::floor((beta_max - beta) / bin_size));
                        if (row > image_width - 2 || row < 0) {
                            continue;
                        }

                        // Bilinear interpolation
                        auto a = alpha / bin_size - col;
                        auto b = beta_max / bin_size - beta / bin_size - row;
                        EASSERT(a <= 1.0 && a >= 0.0);
                        EASSERT(b <= 1.0 && b >= 0.0);
                        spin_img(row * image_width + col, ii) += (1.0f - a) * (1.0f - b);
                        spin_img(row * image_width + col + 1, ii) += a * (1.0f - b);
                        spin_img((row + 1) * image_width + col, ii) += (1.0f - a) * b;
                        spin_img((row + 1) * image_width + col + 1, ii) += a * b;
                    }
                }
            }
        }
    }
}

} // namespace Euclid
