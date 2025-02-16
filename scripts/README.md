# Scripts

All scripts assume repository root as working directory (they resolve paths relative to `scripts/..`).

## Environment setup

| Script | Purpose |
|--------|---------|
| `install_deps_macos.sh` | Homebrew: `cmake ninja eigen@3`, plus an Xcode Command Line Tools check |
| `install_deps_ubuntu.sh` | apt: `build-essential cmake ninja-build git libeigen3-dev` |

Both take `--dry-run`. Between them they install everything the build links
against, so the configure itself needs no network. Neither installs CUDA or
ROS 2, which are large opt-ins with their own instructions.

Builds land **beside** the repository, one directory per configuration:
`../<repo>-build`, `../<repo>-build-debug`, and so on: the default
configuration gets the bare name and every departure from it adds a tag. Out of
tree so `git status` never has to look past build output. One per configuration
so switching between build types is not a full rebuild. `CAMLOC_BUILD_DIR`
overrides the scheme.

`lib.sh` is not run directly: it holds the shared helpers (`camloc_nproc`,
`camloc_build_dir`) that the scripts above source.
