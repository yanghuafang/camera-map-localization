add_library(cam_loc_core
  core/math.cc
)

target_include_directories(cam_loc_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cam_loc_core
  PUBLIC
    Eigen3::Eigen
)
