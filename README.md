# Euclid (Mac M1 Port)

This repository is a **port of the original [Euclid](https://github.com/unclejimbo/Euclid) library** to Apple Silicon (Mac M1/M2/M3) and macOS.

## Important Disclaimer

**I take no credit whatsoever for the original researcher's work.** All foundational research, algorithms, and the core implementation of this library were developed by [unclejimbo](https://github.com/unclejimbo) and the original contributors. Please refer to the [original repository](https://github.com/unclejimbo/Euclid) and [official documentation](https://unclejimbo.github.io/Euclid/) for the primary source of truth.

## Mac M1 Port Contributions

The primary focus of this fork is to ensure that Euclid remains accessible and functional on modern Apple Silicon hardware. The contributions made in this port include:

- **Apple Silicon Compatibility:** Resolved architecture-specific issues to ensure seamless compilation and execution on ARM64 (M1/M2/M3 chips).
- **Build System Updates:** Updated the CMake configuration to correctly detect and link dependencies installed via Homebrew on macOS.
- **Dependency Management:** Fine-tuned dependency resolution for macOS-specific paths (Boost, CGAL, Eigen, Libigl, etc.).
- **Performance Verification:** Verified the library's functionality and performance within the macOS ecosystem.

---

## Introduction

Euclid is a header-only library for geometry processing and shape analysis. It contains utilities and algorithms that extend and cooperate with popular libraries like Eigen (libigl) and CGAL.

The purpose of Euclid is to glue these packages together and provide additional algorithms that are not available in those libraries.

## Dependencies

Different packages in Euclid may require different sets of dependencies. While some are dependency-free (like the IO module), most rely on:

- [Boost](https://www.boost.org/)
- [CGAL](https://www.cgal.org/index.html)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Libigl](http://libigl.github.io/libigl/)

Additional dependencies:
- [Spectra](https://spectralib.org/) (eigenvalue problems)
- [Embree](http://embree.github.io) (CPU ray tracing)
- [Vulkan](https://www.vulkan.org/) (headless GPU rendering)
- [TTK](https://topology-tool-kit.github.io/) (topological shape analysis)
- [Cereal](http://uscilab.github.io/cereal/index.html) (serialization)

**MacOS Note:** These can be easily installed via [Homebrew](https://brew.sh/):
```bash
brew install boost cgal eigen libigl
```

## Installation

Euclid is a header-only library, so CMake configuration is not strictly mandatory, but it is recommended for managing dependencies.

### Using CMake

1. **Option A: find_package**
   ```cmake
   find_package(Euclid)
   target_include_directories(your-target Euclid_INCLUDE_DIR)
   ```

2. **Option B: Config Mode**
   Configure Euclid first to generate `EuclidConfig.cmake`. Then:
   ```cmake
   find_package(Euclid)
   target_link_libraries(your-target Euclid::Euclid)
   ```

## Getting Started

```cpp
#include <Euclid/IO/OffIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/Geometry/TriMeshGeometry.h>
// ... see examples/ for full usage
```

## Examples

Check the [examples](https://github.com/csv610/Euclid/tree/master/examples) folder for tutorials and the [test cases](https://github.com/csv610/Euclid/tree/master/test) for complete usage of functions and classes.

## License

MIT for code not related to third-party libraries. Otherwise, it follows the license requirements of the respective third-party libraries.
