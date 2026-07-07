include(GoogleTest)

add_executable(cam_loc_tests
  test_math.cpp
  test_calib_parser.cpp
  test_pose_reader.cpp
  test_perception_json.cpp
  test_corridor_map.cpp
  test_osm_map.cpp
  test_distance_transform.cpp
  test_pose_sampler.cpp
  test_localization_engine.cpp
  test_semantic_kitti.cpp
  test_semantic_lidar.cpp
  test_perception_eval.cpp
  test_benchmark.cpp
  test_cuda.cpp
  test_viz.cpp
)

target_link_libraries(cam_loc_tests PRIVATE cam_loc_core GTest::gtest_main)
target_compile_definitions(cam_loc_tests PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

gtest_discover_tests(cam_loc_tests)
