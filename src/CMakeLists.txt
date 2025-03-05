add_library(cam_loc_core
  core/math.cc
  kitti/types.cc
  kitti/calib_parser.cc
  kitti/perception_loader.cc
)

target_include_directories(cam_loc_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cam_loc_core
  PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)
