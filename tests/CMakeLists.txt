include(GoogleTest)

add_executable(cam_loc_tests
  math_test.cc
  frames_test.cc
)

target_link_libraries(cam_loc_tests PRIVATE cam_loc_core GTest::gtest_main)

gtest_discover_tests(cam_loc_tests)
