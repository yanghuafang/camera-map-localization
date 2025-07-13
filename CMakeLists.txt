cmake_minimum_required(VERSION 3.18)

project(camera_map_localization VERSION 0.1.0 LANGUAGES CXX)

# Default to Release. Left unset, CMAKE_BUILD_TYPE is empty on single-config
# generators, which means no -O flag at all -- not a debug build, an unoptimized
# one. The pose grid is a triple loop over Eigen expressions, and unoptimized
# Eigen inlines nothing: the smoke sequence runs ~200x slower that way, slow
# enough to be mistaken for the algorithm's own cost. Multi-config generators
# choose per build instead, so there is nothing to default there.
get_property(camloc_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT camloc_multi_config AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
  Debug Release RelWithDebInfo MinSizeRel)

# CUDA targets NVIDIA GPUs and is unavailable on macOS, so default the option off on
# Apple platforms for a clean out-of-the-box configure. It stays on elsewhere (Linux),
# where it still falls back to CPU stubs when nvcc is missing.
if(APPLE)
  set(CAMLOC_BUILD_CUDA_DEFAULT OFF)
else()
  set(CAMLOC_BUILD_CUDA_DEFAULT ON)
endif()

option(CAMLOC_BUILD_CUDA "Build CUDA kernels" ${CAMLOC_BUILD_CUDA_DEFAULT})
option(CAMLOC_BUILD_TESTS "Build unit tests" ON)

# Type-check the GPU host paths without nvcc or a GPU. Everything inside
# #ifdef CAMLOC_CUDA_ENABLED is invisible to a CPU-only build, so a rename can
# leave those call sites behind while CI stays green -- which is exactly how
# they drifted out of sync with cuda/distance_transform.h. With this on, the
# stub is linked but the macro is still defined, so those blocks are compiled
# and checked; at run time IsAvailable() returns false and every path falls back
# to the CPU, so the tests and the smoke benchmark behave as they do CPU-only.
option(CAMLOC_CUDA_HOST_ONLY
  "Compile the CUDA host paths against the CPU stub (no nvcc needed)" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies come from the system package manager -- Homebrew on macOS, apt on
# Ubuntu -- installed by scripts/install_deps_{macos,ubuntu}.sh.
#
# There is deliberately no fetch-at-configure fallback. Cloning these four
# repositories cost a couple of minutes and a quarter-gigabyte for every build
# directory created, and turned a blocked or throttled route to github.com into
# a wall of CMake internals ending in "Configuring incomplete". Requiring the
# packages means the configure touches the network never, takes under a second,
# and when something is missing says which script installs it.

# Homebrew keeps versioned formulae keg-only: installed, but not linked into the
# prefix CMake searches. eigen@3 is the one that matters -- the unversioned
# `eigen` formula is 5.x, and Eigen's own version file declares 5.x incompatible
# with a request for 3.4. scripts/lib.sh resolves the llvm keg the same way.
if(APPLE)
  find_program(CAMLOC_BREW brew)
  if(CAMLOC_BREW)
    execute_process(COMMAND "${CAMLOC_BREW}" --prefix eigen@3
                    OUTPUT_VARIABLE camloc_eigen_keg
                    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(camloc_eigen_keg AND IS_DIRECTORY "${camloc_eigen_keg}")
      list(APPEND CMAKE_PREFIX_PATH "${camloc_eigen_keg}")
    endif()
  endif()
endif()

set(camloc_missing "")

# NO_CMAKE_PACKAGE_REGISTRY: Eigen's own CMakeLists calls export(PACKAGE Eigen3),
# so any build tree that ever fetched Eigen registers itself in ~/.cmake/packages
# and would be found here as though it were installed -- including build trees
# that have since been deleted.
find_package(Eigen3 3.4 QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(NOT TARGET Eigen3::Eigen)
  list(APPEND camloc_missing "Eigen >= 3.4   (brew: eigen@3, apt: libeigen3-dev)")
endif()

find_package(nlohmann_json 3.11 QUIET)
if(NOT TARGET nlohmann_json::nlohmann_json)
  list(APPEND camloc_missing "nlohmann/json  (brew: nlohmann-json, apt: nlohmann-json3-dev)")
endif()

# stb is header-only with no package config anywhere, so it is found by header
# rather than by config file. PATH_SUFFIXES stb because apt's libstb-dev
# installs under /usr/include/stb; HINTS because Homebrew has no stb formula at
# all and install_deps_macos.sh clones it beside the repository instead.
find_path(CAMLOC_STB_INCLUDE_DIR
  NAMES stb_image.h
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../camera-map-localization-deps/stb"
  PATH_SUFFIXES stb)
if(NOT CAMLOC_STB_INCLUDE_DIR)
  list(APPEND camloc_missing "stb            (apt: libstb-dev; macOS: install_deps_macos.sh clones it)")
endif()

if(CAMLOC_BUILD_TESTS)
  find_package(GTest QUIET)
  if(NOT TARGET GTest::gtest_main)
    list(APPEND camloc_missing "GoogleTest     (brew: googletest, apt: libgtest-dev)")
  endif()
endif()

if(camloc_missing)
  string(REPLACE ";" "\n  " camloc_missing_lines "${camloc_missing}")
  message(FATAL_ERROR
    "Missing dependencies:\n  ${camloc_missing_lines}\n\n"
    "Install them with:\n"
    "  ./scripts/install_deps_macos.sh     (macOS)\n"
    "  ./scripts/install_deps_ubuntu.sh    (Ubuntu)\n")
endif()

# Say what was found. There is one route now, but the versions still differ
# between machines -- apt has Eigen 3.4.0 where Homebrew has 3.4.1 -- and a
# build that behaves differently on two hosts starts here.
message(STATUS "Eigen ${Eigen3_VERSION}, nlohmann/json ${nlohmann_json_VERSION}, "
               "stb ${CAMLOC_STB_INCLUDE_DIR}")
# Datasets -- downloaded KITTI, generated smoke sequences, perception JSON --
# live beside the repository, the same way build directories do, because they
# are large, regenerable, and not ours to version. This mirrors camloc_data_dir
# in scripts/lib.sh; keep the two in step.
get_filename_component(camloc_parent "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(camloc_name "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
set(CAMLOC_DATA_DIR "${camloc_parent}/${camloc_name}-data" CACHE PATH
    "Directory holding the datasets (smoke_kitti/, kitti_odometry/, perception/)")

if(CAMLOC_BUILD_TESTS)
  message(STATUS "GoogleTest ${GTest_VERSION}, data ${CAMLOC_DATA_DIR}")
  enable_testing()
endif()

# Position matters: add_compile_options() applies to targets created after it,
# so these flags reach cam_loc's own code below and nothing else -- the
# dependencies arrive as imported targets carrying their own interface flags.
# distance_transform_kernels.cu is compiled by an explicit nvcc command rather
# than by this target, so src/cuda/CMakeLists.txt passes the same two flags
# through -Xcompiler itself.
add_compile_options(-Wall -Wextra)

# Opt-in and not gated in CI: the tree is clean under -Werror here, but -Wall
# differs between compilers and releases, so gating would turn a toolchain
# upgrade into a red build on unrelated pull requests.
option(CAMLOC_WERROR "Treat compiler warnings as errors" OFF)
if(CAMLOC_WERROR)
  add_compile_options(-Werror)
endif()

# Sanitizers, as a comma-separated -fsanitize= list. Compile and link flags have
# to agree -- an instrumented object linked without the runtime fails at link
# time -- so both are set here. Eigen, nlohmann/json and stb are header-only and
# are instrumented along with the code that includes them; GoogleTest is a
# prebuilt library and is not. That is fine: the runtime still checks every
# instrumented translation unit, and cam_loc's own code is what these are
# pointed at.
set(CAMLOC_SANITIZER "" CACHE STRING
  "Sanitizers to enable, e.g. address, undefined, or address,undefined")
if(CAMLOC_SANITIZER)
  add_compile_options(-fsanitize=${CAMLOC_SANITIZER} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${CAMLOC_SANITIZER})
  # Sanitizer builds are optimized (see scripts/ci.sh), and an optimized build
  # type defines NDEBUG, which switches off Eigen's own bounds and dimension
  # assertions -- exactly the class of bug the sanitizers are here for. Undefine
  # it again. Directory options are appended after the per-config flags, so this
  # wins over the -DNDEBUG that CMAKE_CXX_FLAGS_<CONFIG> supplies.
  add_compile_options(-UNDEBUG)
  # UBSan logs and keeps going by default, so a finding reads as a stray line in
  # a passing run. Trapping turns it into an exit code, which is the only form
  # CI can act on.
  if(CAMLOC_SANITIZER MATCHES "undefined")
    add_compile_options(-fno-sanitize-recover=undefined)
  endif()
endif()

# Source-based coverage instrumentation, for scripts/coverage.sh. Clang only:
# GCC's --coverage writes a different format that llvm-cov cannot read, and
# carrying both report paths would double the machinery for one number.
option(CAMLOC_COVERAGE "Instrument for source-based coverage (Clang)" OFF)
if(CAMLOC_COVERAGE)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR
      "CAMLOC_COVERAGE needs Clang; this is ${CMAKE_CXX_COMPILER_ID}.\n"
      "  Re-run with: CXX=clang++ ./scripts/coverage.sh")
  endif()
  add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
  add_link_options(-fprofile-instr-generate)
endif()

add_subdirectory(src)
add_subdirectory(apps)

if(CAMLOC_BUILD_TESTS)
  add_subdirectory(tests)
endif()
