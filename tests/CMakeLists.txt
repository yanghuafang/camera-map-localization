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
  cost_quadratic_test.cc
  cost_modes_test.cc
)

target_link_libraries(cam_loc_tests PRIVATE cam_loc_core GTest::gtest_main)
# TEST_DATA_DIR is the committed fixtures in tests/data.
target_compile_definitions(cam_loc_tests PRIVATE
  TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

gtest_discover_tests(cam_loc_tests)
