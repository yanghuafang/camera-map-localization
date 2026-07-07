add_library(cam_loc_core
  core/math.cpp
  core/localization_kf.cpp
  core/localization_engine.cpp
  core/distance_transform_cpu.cpp
  core/projection.cpp
  core/cost_grid.cpp
  core/pose_sampler.cpp
  core/cost_aggregator.cpp
  core/sampling_covariance.cpp
  kitti/types.cpp
  kitti/calib_parser.cpp
  kitti/perception_loader.cpp
  map/polyline_map.cpp
  map/map_georef.cpp
  map/osm_xml_parser.cpp
  map/osm_map_loader.cpp
  map/map_loader_util.cpp
  map/trajectory_corridor_map.cpp
  perception/adapter.cpp
  perception/synthesize.cpp
  perception/noise.cpp
  perception/resolve.cpp
  kitti/sequence_eval.cpp
  kitti/sequence_eval_runner.cpp
  benchmark/benchmark_runner.cpp
  benchmark/micro_benchmarks.cpp
  viz/frame_viz.cpp
  semantic_kitti/preprocess.cpp
  semantic_kitti/lidar_project.cpp
)

target_include_directories(cam_loc_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${stb_SOURCE_DIR}
)

target_link_libraries(cam_loc_core
  PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)

add_subdirectory(cuda)

target_link_libraries(cam_loc_core PUBLIC cam_loc_cuda)
if(CAMLOC_HAS_CUDA)
  target_compile_definitions(cam_loc_core PUBLIC CAMLOC_CUDA_ENABLED)
endif()
