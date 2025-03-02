# Scripts

All scripts assume repository root as working directory (they resolve paths relative to `scripts/..`).

## Environment setup

| Script | Purpose |
|--------|---------|
| `install_deps_macos.sh` | Homebrew: `cmake ninja eigen@3 googletest llvm`, plus an Xcode Command Line Tools check. `--groups build,style,data` installs a subset; CI takes only what each job uses |
| `install_deps_ubuntu.sh` | apt: `build-essential cmake ninja-build git libeigen3-dev libgtest-dev clang-format curl unzip`. `--groups build,style,data` installs a subset; CI takes only what each job uses |

Both take `--dry-run`. Between them they install everything the build links
against, so the configure itself needs no network. Neither installs CUDA or
ROS 2, which are large opt-ins with their own instructions.

## Build and test

| Script | Purpose |
|--------|---------|
| `ci.sh` | Format, configure, build and run the unit tests. `--debug` / `--release` select the build under test; `--no-style` skips the format gate |

Builds land **beside** the repository, one directory per configuration:
`../<repo>-build`, `../<repo>-build-debug`, and so on: the default
configuration gets the bare name and every departure from it adds a tag. Out of
tree so `git status` never has to look past build output. One per configuration
so switching between build types is not a full rebuild. `CAMLOC_BUILD_DIR`
overrides the scheme.

## Style gates

| Script | Purpose |
|--------|---------|
| `format.sh` | `clang-format` over `src/`, `include/` and `tests/`, plus a trailing-whitespace strip that also covers the scripts, docs and CMakeLists; `--check` reports without writing |

It is run by `ci.sh` and by CI:

```bash
brew install llvm              # macOS — Xcode ships it not
sudo apt install clang-format  # Ubuntu
```

`lib.sh` is not run directly: it holds the shared helpers (`camloc_nproc`, the
clang-tool resolver, the source-file list) that the scripts above source.

## Data preparation

| Script | Purpose |
|--------|---------|
| `download_kitti_odometry.sh [dest]` | Fetch poses + calib zips into `<repo>-data/kitti_odometry/`; `--force` re-fetches |
