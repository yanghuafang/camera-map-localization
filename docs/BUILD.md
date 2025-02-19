# Build guide

## Project naming

| Layer | Name |
|-------|------|
| GitHub repository | `camera-map-localization` |
| CMake project | `camera_map_localization` |
| C++ namespace / headers | `cam_loc` under `include/cam_loc/` |
| Static library | `cam_loc_core` |
| CMake options | `CAMLOC_*` |

The public repo name reflects **camera map localization**; internal `cam_loc` identifiers are kept for brevity and API stability.

## System requirements

| Component | Version |
|-----------|---------|
| Operating system | Linux or macOS (Intel or Apple Silicon) |
| CMake | ≥ 3.18 |
| C++ compiler | C++17 (GCC 9+, Clang 10+, or Apple Clang 12+) |

### Platform notes

- **Linux** is the primary target.
- **macOS** builds with the stock Apple Clang toolchain (`xcode-select --install`).
- The `scripts/*.sh` helpers detect the CPU count portably (`nproc` on Linux, `getconf`/`sysctl` on macOS), so they run unchanged on both platforms.

### Installing them

From a clean machine:

```bash
./scripts/install_deps_macos.sh     # Homebrew
./scripts/install_deps_ubuntu.sh    # apt
```

Both take `--dry-run` to print the package list and exit.

## Third-party dependencies

Resolved from the **system package manager**:

| Library | Homebrew | apt | Used for |
|---------|----------|-----|----------|
| Eigen | `eigen@3` 3.4.1 ✓ | `libeigen3-dev` 3.4.0 ✓ | Linear algebra |
| GoogleTest | `googletest` 1.18 ✓ | `libgtest-dev` 1.17 ✓ | Unit tests |

With them present the configure needs **no network** — on Ubuntu it drops from
minutes to under a second — and a blocked or throttled route to github.com stops
being a build failure.

**On macOS it is `eigen@3`, not `eigen`.** The unversioned formula is 5.x, and
Eigen's own version file declares 5.x incompatible with a request for 3.4, so
CMake declines it rather than silently building against an untested major
version:

```
Could not find a configuration file for package "Eigen3" that is compatible
with requested version "3.4".
  ... version: 5.0.1
  The version found is not compatible with the version requested.
```

`eigen@3` is keg-only, so CMakeLists adds its prefix to `CMAKE_PREFIX_PATH`.

### When a dependency is missing

There is no fetch-at-configure fallback: the configure fails and names the
script that fixes it.

```
CMake Error at CMakeLists.txt:...
  Missing dependencies:
    Eigen >= 3.4   (brew: eigen@3, apt: libeigen3-dev)

  Install them with:
    ./scripts/install_deps_macos.sh     (macOS)
    ./scripts/install_deps_ubuntu.sh    (Ubuntu)
```

The configure reports what it resolved, because the versions differ between
machines:

```
-- Eigen 3.4.1
-- GoogleTest 1.18.0
```

## Configure and build

`$(getconf _NPROCESSORS_ONLN)` reports the core count on both Linux and macOS (substitute `$(nproc)` on Linux if you prefer).

The script picks the build directory for you, which is the usual way in:

```bash
./scripts/ci.sh            # configure, build, test
```

By hand, naming the directory yourself:

```bash
B=../camera-map-localization-build
cmake -S . -B "$B" -DCAMLOC_BUILD_TESTS=ON
cmake --build "$B" -j"$(getconf _NPROCESSORS_ONLN)"
```

### Build directories

Builds land **beside** the repository, one directory per configuration:

```
camera-map-localization/                     the repository
camera-map-localization-build/               the default configuration
camera-map-localization-build-debug/         -DCMAKE_BUILD_TYPE=Debug
```

The default configuration gets the bare name and every departure from it adds a
tag, so two builds that differ only in type still land in different directories.

Out of tree so that a `git status` never has to look past build output, and so
that deleting a configuration is `rm -rf` on something that is not the working
tree. One per configuration so switching between build types is not a full
rebuild.

`CAMLOC_BUILD_DIR` overrides the scheme. `scripts/lib.sh` (`camloc_build_dir`)
is where it is implemented.

### CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `CAMLOC_BUILD_TESTS` | `ON` | Build `cam_loc_tests` and register CTest targets |

The tree builds warning-free under `-Wall -Wextra`, which is always on for
cam_loc's own targets (the dependencies arrive as imported targets and keep
their own settings).

### Targets produced

| Target | Type |
|--------|------|
| `cam_loc_core` | Static library — shared types and SE(3) math |
| `cam_loc_tests` | GoogleTest binary under `<build dir>/tests/` |

## Troubleshooting

- **`Missing dependencies:` at configure time:** run `./scripts/install_deps_macos.sh` or `./scripts/install_deps_ubuntu.sh`. The message lists each missing library with its Homebrew and apt package name.
- **CMake from another directory:** always pass the source tree explicitly, e.g. `cmake -S . -B <dir>` (all `scripts/*.sh` do this).
