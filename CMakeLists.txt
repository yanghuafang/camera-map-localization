cmake_minimum_required(VERSION 3.18)

project(camera_map_localization VERSION 0.1.0 LANGUAGES CXX)

option(CAMLOC_BUILD_TESTS "Build unit tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies come from the system package manager -- Homebrew on macOS, apt on
# Ubuntu -- installed by scripts/install_deps_{macos,ubuntu}.sh.
#
# There is deliberately no fetch-at-configure fallback. Cloning a dependency
# costs time and disk for every build directory created, and turns a blocked or
# throttled route to github.com into a wall of CMake internals ending in
# "Configuring incomplete". Requiring the packages means the configure touches
# the network never, takes under a second, and when something is missing says
# which script installs it.

# Homebrew keeps versioned formulae keg-only: installed, but not linked into the
# prefix CMake searches. eigen@3 is the one that matters -- the unversioned
# `eigen` formula is 5.x, and Eigen's own version file declares 5.x incompatible
# with a request for 3.4.
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
message(STATUS "Eigen ${Eigen3_VERSION}, nlohmann/json ${nlohmann_json_VERSION}")

if(CAMLOC_BUILD_TESTS)
  message(STATUS "GoogleTest ${GTest_VERSION}")
  enable_testing()
endif()

# Position matters: add_compile_options() applies to targets created after it,
# so these flags reach cam_loc's own code below and nothing else -- the
# dependencies arrive as imported targets carrying their own interface flags.
add_compile_options(-Wall -Wextra)

# Opt-in and not gated in CI: the tree is clean under -Werror here, but -Wall
# differs between compilers and releases, so gating would turn a toolchain
# upgrade into a red build on unrelated pull requests.
option(CAMLOC_WERROR "Treat compiler warnings as errors" OFF)
if(CAMLOC_WERROR)
  add_compile_options(-Werror)
endif()

add_subdirectory(src)

if(CAMLOC_BUILD_TESTS)
  add_subdirectory(tests)
endif()
