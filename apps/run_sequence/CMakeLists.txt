add_executable(run_sequence main.cpp)
target_link_libraries(run_sequence PRIVATE cam_loc_core cam_loc_cuda)
