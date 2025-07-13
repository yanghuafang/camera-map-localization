# Header-only helpers shared by the CLI front-ends. Exists to put the repository
# root on the include path, so "apps/common/arg_parse.h" is a path from the
# source root as Google style requires.
add_library(cam_loc_app_common INTERFACE)
target_include_directories(cam_loc_app_common INTERFACE ${CMAKE_SOURCE_DIR})

add_subdirectory(run_sequence)
add_subdirectory(eval_sequence)
add_subdirectory(eval_perception_compare)
add_subdirectory(benchmark)
add_subdirectory(preprocess_kitti)
add_subdirectory(viz_frame)
