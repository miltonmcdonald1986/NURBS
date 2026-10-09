# NURBS

[![Build](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/build.yml)
[![Tests](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/tests.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/tests.yml)
[![Static Analysis](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/static-analysis.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/static-analysis.yml)
[![Compiler Warnings](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/compiler-warnings.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/compiler-warnings.yml)
[![Dependencies](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/dependencies.yml/badge.svg?branch=main)](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/dependencies.yml)
[![Coverage](https://codecov.io/gh/miltonmcdonald1986/NURBS/branch/main/graph/badge.svg)](https://codecov.io/gh/miltonmcdonald1986/NURBS)
[![Docs](https://github.com/miltonmcdonald1986/NURBS/actions/workflows/docs.yml/badge.svg?branch=main)](https://miltonmcdonald1986.github.io/NURBS/)

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

Pass `-DNURBS_BUILD_VIEWER=ON` to also build `NURBS.Viewer`, an interactive viewer built on GLFW, OpenGL 3.3, and Dear ImGui. On Linux, it needs the OpenGL and X11/Wayland development packages that GLFW requires (for example, `libgl-dev` and `xorg-dev` on Debian/Ubuntu).

### Presets

`CMakePresets.json` defines `x64-debug` and `x64-release` (MSVC), `linux-debug` (GCC), and `macos-debug` (AppleClang). Each one builds the library, the tests, and the viewer with Ninja into `out/build/<preset>`. On Windows, run these from a Developer PowerShell for VS (x64):

```sh
cmake --preset x64-debug
cmake --build --preset x64-debug
ctest --preset x64-debug
```

In Visual Studio, use **File → Open → Folder** on the repository root. Then pick a preset from the configuration dropdown and `NURBS.Viewer.exe` from the startup item dropdown.

### Documentation

The [API reference](https://miltonmcdonald1986.github.io/NURBS/) is generated with [Doxygen](https://www.doxygen.nl/) and published from `main`. To build it locally, run `doxygen` from the repository root and open `build-docs/html/index.html`.

Every public entity in `NURBS.Lib/include` needs a Doxygen comment, including each parameter and return value. Doxygen treats a missing comment as an error, so the [Docs](.github/workflows/docs.yml) workflow fails on undocumented code.

To use the library from another CMake project, add it as a subdirectory and link `NURBS::Lib`.

## Viewer

`NURBS.Viewer` shows the library's algorithms one scene at a time. Pick a scene from the **Scene** combo at the top of the Controls panel. In the main view, right- or middle-drag to pan and use the mouse wheel to zoom.

| Scene | Contents |
|---|---|
| Horner (A1.1) | A 2D power-basis curve `C(u) = Σ a[i] uⁱ` with presets and a degree slider. The coefficients appear as vectors in a side view, and you can drag their tips to edit them. A `u0` slider moves the evaluation point. You can step through the Horner chain `C_n = a_n, C_k = u0·C_{k+1} + a_k` and see a table of its steps. You can also overlay the power-sum terms `u0ⁱ a[i]`, drawn tip to tail, which reach the same point `C(u0)`. |
| Bernstein (A1.2, A1.3) | The graphs of the degree-n Bernstein basis `B_{i,n}(u)` on `[0, 1]`, sampled with `AllBernstein`, plus a degree slider. A `u0` slider moves an evaluation line across the graphs. A table compares `AllBernstein` with `Bernstein` at `u0` and sums the values to 1. A stacked bar shows that partition of unity. You can highlight one basis function to overlay its graph from `Bernstein`. |
| Bezier (A1.4, A1.5) | A 2D Bezier curve built with `BezierCurve`, with presets and a degree slider. Left-drag the control points `P[i]` in the view, or edit them in the panel, to reshape the curve. You can show or hide the control polygon. A `u0` slider moves the point `C(u0)` along the curve. A table compares `BezierCurve::Evaluate` (de Casteljau, A1.5) with `PointOnBezierCurve` (A1.4) at `u0`. |
| Rational Bezier (Eq. 4.1) | A 2D rational Bezier curve built with `RationalBezierCurve`, with presets (conic arcs, a quarter circle, a semicircle) and a degree slider. Left-drag the control points `P[i]` in the view, or edit them and their weights `w[i]` in the panel, to reshape the curve. You can label the points with their weights and overlay the polynomial curve with every weight 1. A `u0` slider moves the point `C(u0)` along the curve. A table compares `RationalBezierCurve::Evaluate` (de Casteljau in homogeneous space) with Eq. 4.1 evaluated from `AllBernstein` at `u0`. For degree 2 the panel names the conic type from the weights. |

## Dependencies

The library itself has no dependencies. The tests fetch the following with `FetchContent`, pinned by SHA256:

- [GoogleTest](https://github.com/google/googletest) v1.18.0
- [GLM](https://github.com/g-truc/glm) 1.0.3

The viewer also fetches:

- [GLFW](https://github.com/glfw/glfw) 3.5.1
- [Dear ImGui](https://github.com/ocornut/imgui) v1.92.9b

A weekly [Dependencies](.github/workflows/dependencies.yml) workflow flags pins that fall behind the latest upstream release, and Dependabot keeps the GitHub Actions up to date.

## License

[MIT](LICENSE)
