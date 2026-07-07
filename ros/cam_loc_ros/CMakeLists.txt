cmake_minimum_required(VERSION 3.18)
project(cam_loc_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# cam_loc_core links the CUDA runtime when built with CAMLOC_BUILD_CUDA=ON. Link cudart here too
# when the toolkit is present. Optional so a CPU-only core (stub cam_loc_cuda) still builds.
find_package(CUDAToolkit QUIET)
set(CAMLOC_CUDA_RUNTIME "")
if(TARGET CUDA::cudart)
  set(CAMLOC_CUDA_RUNTIME CUDA::cudart)
endif()

set(CAMLOC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "camera-map-localization repository root")
set(CAMLOC_BUILD_DIR "${CAMLOC_ROOT}/build" CACHE PATH "camera-map-localization CMake build directory")

find_library(CAMLOC_CORE_LIB cam_loc_core PATHS "${CAMLOC_BUILD_DIR}/src" NO_DEFAULT_PATH REQUIRED)
find_library(CAMLOC_CUDA_LIB cam_loc_cuda PATHS "${CAMLOC_BUILD_DIR}/src/cuda" NO_DEFAULT_PATH REQUIRED)

add_library(cam_loc_ros_markers src/markers.cpp)
target_include_directories(cam_loc_ros_markers
  PUBLIC
    include
    "${CAMLOC_ROOT}/include"
    "${CAMLOC_BUILD_DIR}/_deps/stb-src"
    "${CAMLOC_BUILD_DIR}/_deps/eigen3-src"
    "${CAMLOC_BUILD_DIR}/_deps/nlohmann_json-src/include"
)
# Modern target style (ament_target_dependencies was removed in newer ament_cmake).
target_link_libraries(cam_loc_ros_markers PUBLIC
  ${CAMLOC_CORE_LIB} ${CAMLOC_CUDA_LIB} ${CAMLOC_CUDA_RUNTIME}
  rclcpp::rclcpp
  ${visualization_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(cam_loc_viz_node src/loc_viz_node.cpp src/keyboard_input.cpp)
target_include_directories(cam_loc_viz_node PRIVATE
  include
  "${CAMLOC_ROOT}/include"
  "${CAMLOC_BUILD_DIR}/_deps/stb-src"
  "${CAMLOC_BUILD_DIR}/_deps/eigen3-src"
  "${CAMLOC_BUILD_DIR}/_deps/nlohmann_json-src/include"
)
target_link_libraries(cam_loc_viz_node
  cam_loc_ros_markers
  ${CAMLOC_CORE_LIB} ${CAMLOC_CUDA_LIB} ${CAMLOC_CUDA_RUNTIME}
  rclcpp::rclcpp
  ${visualization_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS cam_loc_viz_node cam_loc_ros_markers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
