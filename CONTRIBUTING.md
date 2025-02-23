# Contributing

Thank you for contributing to **camera-map-localization**. This project is a clean-room C++ implementation of camera map-matching localization for KITTI; it must remain free of proprietary automotive SDK code.

## Development setup

```bash
git clone <your-fork-url>
cd camera-map-localization

./scripts/install_deps_macos.sh    # or install_deps_ubuntu.sh
./scripts/ci.sh
```

`ci.sh` is the gate set: format, build, tests. Builds land beside the
repository, one directory per configuration — see
[docs/BUILD.md](docs/BUILD.md#build-directories).

The style gate needs `clang-format`:

```bash
brew install llvm              # macOS — Xcode ships it not
sudo apt install clang-format  # Ubuntu
```

Without it, `./scripts/ci.sh --no-style` runs the build and tests alone.

## Pull requests

1. **Branch** from `main`.
2. **Scope** — Keep changes focused. Separate unrelated fixes into different PRs when possible.
3. **Tests** — Add or update GoogleTest coverage for new behavior.
4. **Docs** — Update the relevant guide under `docs/` and `README.md` if user-facing behavior, CLI flags, or data layout changes.
5. **Scripts** — If you add a helper script, document it in `scripts/README.md`.
6. **CI** — PRs must pass two workflows: [`Lint`](.github/workflows/lint.yml) (`clang-format`)
   and [`Build`](.github/workflows/build.yml) (`Ubuntu`, `macOS`). Every configuration they
   build is a preset in `CMakePresets.json`, so any red job reproduces locally with the same
   commands — e.g. `cmake --preset cpu`, `cmake --build --preset cpu`, `ctest --preset cpu`.

## Commit messages

Write them as **Conventional Commits** — `type(scope): description`, lowercase, imperative, no trailing period. The types in use are `feat`, `fix`, `refactor`, `perf`, `docs`, `test`, `build` and `ci`; the scope is the subsystem the change lands in (`core`, `map`, `perception`, `kitti`, `scripts`). For example:

```
feat(core): search the pose grid on vehicle axes
```

Use the body to say **why**, not what — the diff already says what. Wrap it at 72 columns and prefer a few bullets, one per decision, each carrying the reason it went that way and what it cost:

- What was wrong before, in terms a reader can check against the code.
- Why this shape and not the obvious alternative.
- What moved as a consequence — a threshold, a doc claim that stopped being true.

Keep unrelated changes out of the same commit.

## Code guidelines

This project follows the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Most of it is enforced rather than described — `./scripts/format.sh` is the authority. What follows is the part that needs saying anyway.

- **Language:** C++17, no extensions.
- **Formatting:** `clang-format` using the repo's [`.clang-format`](.clang-format) — Google base style, 2-space indent, 80-column limit. Run `./scripts/format.sh` before committing; `--check` is what the gate runs.
- **Files:** sources are `.cc`, headers `.h`, both `lower_case`. A test for `foo.cc` is `foo_test.cc`.
- **Header guards:** `#define` guards, not `#pragma once`, named for the include path — `include/cam_loc/types/status.h` guards with `CAM_LOC_TYPES_STATUS_H_`.
- **Naming:** types and functions `UpperCamelCase`; variables, parameters and members `lower_case`, with a trailing underscore on private members; constants and enumerators `kUpperCamelCase`; namespaces `lower_case`. Two exemptions are deliberate: a trivial accessor may be named for the member it returns (`step_x()`), and a matrix or transform may use geometry notation (`T_world_rig`, `K`, `R0_rect`) rather than being renamed into something that reads as a translation.
- **Includes:** project headers are quoted and spelled as a path from an include root — `#include "cam_loc/types/status.h"`. Order is the guide's: related header, C system, C++ standard library, other libraries, this project. `.clang-format` regroups automatically, so writing them in any order and running `format.sh` is enough.
- **Exceptions:** not used. Errors travel as `cam_loc::Status`.
- **Warnings:** the build is `-Wall -Wextra` and clean.
- **Headers:** Public API under `include/cam_loc/`; implementation in `src/`.
- **Project naming:** repository is **camera-map-localization**; CMake project `camera_map_localization`. Keep the `cam_loc` namespace and the library target names unless doing a deliberate API break.
- **Comments:** Explain **intent and trade-offs** — non-obvious algorithm steps, and why a thing is done the way it is. Do not narrate obvious code line by line.
- **API docs:** Public headers use `///` comments whose first sentence is the brief. Add `@param` / `@return` where a parameter carries a **unit, a frame, or a constraint**, and leave them off where the signature already says it — `@param uv Pixel coordinates` is the narration the previous point rules out.
- **Dependencies:** Prefer something Homebrew and apt both ship, and wire it into `CMakeLists.txt` and *both* `scripts/install_deps_*.sh`; a dependency only one platform can install is a dependency half the readers cannot build. Do not add heavy ones without discussion. Add a third-party include directory as `SYSTEM` so its warnings are not reported as ours.

## What not to commit

- IDE/agent configs (`.cursor/`, `AGENTS.md`).
- Proprietary automotive SDK source, headers, or copied test data.
- Large binary datasets; document download steps instead.

## Reporting issues

Include:

- OS, compiler and CMake versions
- Exact configure/build commands
- Minimal repro
- Relevant log output

## License

By contributing, you agree that your contributions are licensed under the [MIT License](LICENSE).
