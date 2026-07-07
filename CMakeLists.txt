cmake_minimum_required(VERSION 3.18)

project(camera_map_localization VERSION 0.1.0 LANGUAGES CXX)

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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen3)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG master
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(stb)

if(CAMLOC_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    GIT_SHALLOW TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
endif()

add_subdirectory(src)
add_subdirectory(apps)

if(CAMLOC_BUILD_TESTS)
  add_subdirectory(tests)
endif()
