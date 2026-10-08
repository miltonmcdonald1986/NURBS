# NURBS

[![Build](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/build.yml)
[![Tests](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/tests.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/tests.yml)
[![Static Analysis](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/static-analysis.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/static-analysis.yml)
[![Compiler Warnings](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/compiler-warnings.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/compiler-warnings.yml)
[![Dependencies](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/dependencies.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/dependencies.yml)
[![Coverage](https://codecov.io/gh/miltonmcdonald1986/NURBS/branch/main/graph/badge.svg)](https://codecov.io/gh/miltonmcdonald1986/NURBS)

A C++23 NURBS library implementing the algorithms from *The NURBS Book* (Piegl & Tiller).

## Features

All functions live in the `NURBS` namespace and are available through `#include <NURBS/NURBS.hpp>`.

| Header | Contents |
|---|---|
| `NURBS/horner1.hpp` | `horner1`, power-basis curve evaluation with Horner's method (A1.1); the `CurvePoint` concept |
| `NURBS/bernstein.hpp` | `Bernstein`, `AllBernstein` (A1.2, A1.3) |
| `NURBS/bezier.hpp` | `PointOnBezierCurve`, `deCasteljau1`, `BezierCurve` (A1.4, A1.5) |

Curve algorithms are templated on the point type, so scalars and vector types such as `glm::dvec3` work out of the box.

## Requirements

- CMake 3.25 or newer
- A C++23 compiler (CI covers MSVC, GCC, and Clang on Windows, Linux, and macOS)

## Building and testing

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build --build-config Release
```

Pass `-DNURBS_BUILD_TESTS=OFF` to build only the library.

To use the library from another CMake project, add it as a subdirectory and link `NURBS::Lib`.

## Dependencies

The library itself has no dependencies. The tests fetch the following with `FetchContent`, pinned by SHA256:

- [GoogleTest](https://github.com/google/googletest) v1.18.0
- [GLM](https://github.com/g-truc/glm) 1.0.3

A weekly [Dependencies](.github/workflows/dependencies.yml) workflow flags pins that fall behind the latest upstream release, and Dependabot keeps the GitHub Actions up to date.

## License

[MIT](LICENSE)
