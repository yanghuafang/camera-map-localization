cmake_minimum_required(VERSION 3.18)
add_executable(viz_frame main.cpp)
target_link_libraries(viz_frame PRIVATE cam_loc_core)
