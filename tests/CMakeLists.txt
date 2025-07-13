include(GoogleTest)

add_executable(cam_loc_tests
  math_test.cc
  frames_test.cc
  calib_parser_test.cc
  pose_reader_test.cc
  perception_json_test.cc
  corridor_map_test.cc
  osm_map_test.cc
  distance_transform_test.cc
  pose_sampler_test.cc
  localization_kf_test.cc
  localization_engine_test.cc
  semantic_kitti_test.cc
  semantic_lidar_test.cc
  perception_eval_test.cc
  benchmark_test.cc
  cuda_test.cc
  viz_test.cc
)

target_link_libraries(cam_loc_tests PRIVATE cam_loc_core GTest::gtest_main)
# TEST_DATA_DIR is the committed fixtures in tests/data. CAMLOC_DATA_DIR is the
# generated and downloaded datasets, which live outside the repository -- a test
# that needs them skips when they are absent.
target_compile_definitions(cam_loc_tests PRIVATE
  TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
  CAMLOC_DATA_DIR="${CAMLOC_DATA_DIR}")

gtest_discover_tests(cam_loc_tests)
