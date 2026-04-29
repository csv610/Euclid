# Euclid User Guide

Euclid is a header-only C++ library for geometry processing and shape analysis. It extends and integrates with popular libraries like Eigen, libigl, and CGAL to provide additional algorithms for mesh processing, shape analysis, and visualization.

## Table of Contents

1. [Introduction](#introduction)
2. [Installation](#installation)
3. [Quick Start](#quick-start)
4. [Core Concepts](#core-concepts)
5. [Module Overview](#module-overview)
6. [Working with Meshes](#working-with-meshes)
7. [Examples](#examples)

---

## Introduction

Euclid provides a comprehensive set of tools for:

- **Mesh I/O**: Read and write OBJ, OFF, PLY, STL files
- **Mesh Processing**: Bounding volumes, topology analysis, parameterization
- **Shape Analysis**: Descriptors (Spin Images, HKS, WKS), distance metrics
- **Visualization**: Rendering with Embree ray tracing or Vulkan rasterization
- **Utilities**: Memory management, timing, serialization

---

## Installation

### Prerequisites

Install dependencies via Homebrew (macOS) or your preferred package manager:

```bash
# Core dependencies
brew install boost cgal eigen libigl

# Optional dependencies
brew install spectra embree vulkan-sdk
```

### Building

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Running Tests

```bash
./build/bin/test/run_test
```

---

## Quick Start

```cpp
#include <Euclid/IO/ObjIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Geometry/TriMeshGeometry.h>

#include <vector>
#include <string>

using Kernel = CGAL::Simple_cartesian<float>;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

int main() {
    // Read mesh from file
    std::vector<float> positions, normals;
    std::vector<unsigned> indices;
    Euclid::read_obj("mesh.obj", positions, &normals, &indices, nullptr);

    // Create CGAL mesh
    Mesh mesh;
    Euclid::make_mesh(mesh, positions, indices);

    // Compute geometric properties
    auto curvatures = Euclid::gaussian_curvatures(mesh);
    auto areas = Euclid::face_areas(mesh);

    return 0;
}
```

---

## Core Concepts

### Mesh Representation

Euclid supports multiple mesh representations:
- **CGAL Surface_mesh**: Full-featured half-edge data structure
- **Eigen matrices**: Efficient for numerical computation

Convert between representations using:
- `Euclid::make_mesh()` - Eigen vectors to CGAL mesh
- `Euclid::make_Eigen_mesh()` - CGAL mesh to Eigen matrices

### Template Parameters

Most functions are templated:
- `FT`: Floating-point type (float/double)
- `IT`: Index type (int/unsigned)
- `N`: Dimension (typically 3 for 3D meshes)

---

## Module Overview

### IO Module (`<Euclid/IO/*.h>`)

File format support for geometry data:

| Format | Reader | Writer |
|--------|--------|--------|
| OBJ | `read_obj()` | `write_obj()` |
| OFF | `read_off()` | `write_off()` |
| PLY | `read_ply()` | `write_ply()` |
| STL | `read_stl()` | `write_stl()` |

### Math Module (`<Euclid/Math/*.h>`)

- **Vector**: Vector operations (length, normalize, cross, dot)
- **Transformation**: 3D transformations (rotation, translation, scaling)
- **Statistics**: Statistical analysis utilities
- **Numeric**: Numerical helpers

### Geometry Module (`<Euclid/Geometry/*.h>`)

- **TriMeshGeometry**: Differential geometry on triangle meshes
  - Vertex/face normals
  - Curvatures (gaussian, mean)
  - Face areas, edge lengths
  - Laplacian operators

- **Spectral**: Spectral decomposition for shape analysis

### MeshUtil Module (`<Euclid/MeshUtil/*.h>`)

- **CGALMesh**: CGAL mesh utilities
- **EigenMesh**: Eigen matrix utilities
- **MeshHelpers**: Common mesh operations
- **PrimitiveGenerator**: Generate spheres, cubes, planes

### Descriptor Module (`<Euclid/Descriptor/*.h>`)

Shape descriptors for matching and analysis:

- **SpinImage**: Local shape descriptor (Johnson & Hebert 1999)
- **HKS**: Heat Kernel Signature
- **WKS**: Wave Kernel Signature
- **Histogram**: Histogram computation

### Distance Module (`<Euclid/Distance/*.h>`)

Distance metrics between shapes:

- **GeodesicsInHeat**: Approximate geodesic distances
- **BiharmonicDistance**: Biharmonic distance
- **DiffusionDistance**: Diffusion distance

### Parameterization Module (`<Euclid/Parameterization/*.h>`)

Mesh parameterization algorithms:

- **SCP**: Spectral conformal parameterization
- **RicciFlow**: Ricci flow for conformal mapping
- **HolomorphicOneForms**: Holomorphic 1-forms

### Topology Module (`<Euclid/Topology/*.h>`)

- **MeshTopology**: Connectivity and topology queries
- **HomologyGenerator**: Homology basis generation
- **HomotopyGenerator**: Homotopy basis generation
- **Chain**: Chain complexes

### BoundingVolume Module (`<Euclid/BoundingVolume/*.h>`)

- **AABB**: Axis-aligned bounding box
- **OBB**: Oriented bounding box

### Render Module (`<Euclid/Render/*.h>`)

- **RenderCore**: Core rendering utilities
- **RayTracer**: Ray tracing with Embree
- **Rasterizer**: Rasterization with Vulkan

### ViewSelection Module (`<Euclid/ViewSelection/*.h>`)

- **ViewSphere**: View sphere construction
- **ViewEntropy**: Entropy-based view selection
- **ProxyView**: Proxy-based view selection

### Segmentation Module (`<Euclid/Segmentation/*.h>`)

- **RandomWalk**: Random walk segmentation

### SurfaceDelaunay Module (`<Euclid/SurfaceDelaunay/*.h>`)

- **DelaunayMesh**: Surface Delaunay triangulation

### Util Module (`<Euclid/Util/*.h>`)

- **Timer**: Performance timing
- **Memory**: Memory management
- **Color**: Color utilities and colormaps
- **Serialize**: Serialization with Cereal

---

## Working with Meshes

### Reading and Writing

```cpp
// Read mesh
std::vector<float> positions, normals;
std::vector<unsigned> indices;
Euclid::read_obj("input.obj", positions, &normals, &indices, nullptr);

// Write mesh with colors
std::vector<unsigned char> colors;
Euclid::colormap(igl::COLOR_MAP_TYPE_VIRIDIS, values, colors);
Euclid::write_ply("output.ply", positions, nullptr, nullptr, &indices, &colors);
```

### Computing Geometry

```cpp
Mesh mesh;
// ... create mesh ...

// Vertex normals
auto vnormals = Euclid::vertex_normals(mesh);

// Gaussian curvature
auto curvatures = Euclid::gaussian_curvatures(mesh);

// Face areas
auto areas = Euclid::face_areas(mesh);
```

### Mesh Conversions

```cpp
// Eigen vectors to CGAL mesh
Mesh mesh;
Euclid::make_mesh<3>(mesh, positions, indices);

// CGAL mesh to Eigen matrices
std::vector<float> positions_out, normals_out;
std::vector<unsigned> indices_out;
Euclid::make_Eigen_mesh(mesh, positions_out, &normals_out, &indices_out);
```

---

## Examples

### Hello World

Compute and visualize Gaussian curvature:

```bash
cd build && make hello_world
./bin/example/hello_world
```

### Bounding Volumes

```bash
make bounding_volume_example
```

### Geodesic Distances

```bash
make geodesics_example
```

### Primitive Generation

```bash
make primitive_example
```

### Spin Images

```bash
make spin_image_example
```

### View Selection

```bash
make view_selection_example
```

---

## Additional Resources

- [API Documentation](https://unclejimbo.github.io/Euclid/)
- [Test Cases](https://github.com/csv610/Euclid/tree/master/test)
- [Original Repository](https://github.com/unclejimbo/Euclid)