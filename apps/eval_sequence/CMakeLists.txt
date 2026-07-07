cmake_minimum_required(VERSION 3.18)
add_executable(eval_sequence main.cpp)
target_link_libraries(eval_sequence PRIVATE cam_loc_core)
