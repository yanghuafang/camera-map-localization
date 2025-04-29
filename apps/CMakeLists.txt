# Header-only helpers shared by the CLI front-ends. Exists to put the repository
# root on the include path, so "apps/common/arg_parse.h" is a path from the
# source root as Google style requires.
add_library(cam_loc_app_common INTERFACE)
target_include_directories(cam_loc_app_common INTERFACE ${CMAKE_SOURCE_DIR})

add_subdirectory(run_sequence)
