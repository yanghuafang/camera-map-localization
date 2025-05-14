add_library(cam_loc_core
  core/math.cc
  core/random.cc
  core/localization_kf.cc
  core/localization_engine.cc
  core/distance_transform_cpu.cc
  core/projection.cc
  core/cost_grid.cc
  core/cost_quadratic.cc
  core/cost_modes.cc
  core/cost_aggregator.cc
  core/pose_sampler.cc
  core/sampling_covariance.cc
  perception/adapter.cc
  perception/synthesize.cc
  perception/noise.cc
  perception/resolve.cc
  semantic_kitti/preprocess.cc
  kitti/types.cc
  kitti/calib_parser.cc
  kitti/perception_loader.cc
  kitti/sequence_eval.cc
  kitti/egomotion_noise.cc
  kitti/sequence_eval_runner.cc
  map/polyline_map.cc
  map/map_georef.cc
  map/osm_xml_parser.cc
  map/osm_map_loader.cc
  map/map_loader_util.cc
  map/trajectory_corridor_map.cc
)

target_include_directories(cam_loc_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# SYSTEM (-isystem): stb is header-only and its implementation is compiled into
# cam_loc_core, so under -Wall -Wextra its warnings would be reported as ours.
# -Wno-* would have hidden the same problems in cam_loc's own code.
target_include_directories(cam_loc_core SYSTEM PRIVATE ${CAMLOC_STB_INCLUDE_DIR})

target_link_libraries(cam_loc_core
  PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)
