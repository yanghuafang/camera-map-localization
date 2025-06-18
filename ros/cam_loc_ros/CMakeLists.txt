cmake_minimum_required(VERSION 3.18)
project(cam_loc_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Record the directory of every library actually linked in the installed
# binary's rpath. Without it the node links but dies at launch on macOS, where
# ROS comes from a RoboStack prefix that ament does not put on the rpath:
#   dyld: Library not loaded: @rpath/librclcpp.dylib
# The failure is invisible to the build and only shows when the node starts.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

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

# Mirror the default in scripts/lib.sh camloc_build_dir: builds live beside the
# repository, not inside it. scripts/build_ros.sh passes the real one, so this
# only has to be right for a hand-run colcon.
get_filename_component(camloc_root_abs "${CAMLOC_ROOT}" ABSOLUTE)
get_filename_component(camloc_root_name "${camloc_root_abs}" NAME)
get_filename_component(camloc_root_parent "${camloc_root_abs}" DIRECTORY)
set(CAMLOC_BUILD_DIR "${camloc_root_parent}/${camloc_root_name}-build"
    CACHE PATH "camera-map-localization CMake build directory")

# The same system packages the core build uses (see ../../CMakeLists.txt).
# colcon configures this package on its own, so it repeats the lookup rather
# than inheriting anything -- including the two macOS quirks: eigen@3 is
# keg-only, and stb has no formula, so install_deps_macos.sh clones it beside
# the repository. nlohmann/json is deliberately absent here; it is an
# implementation detail of the map loader and never reaches a public header.
if(APPLE)
  find_program(CAMLOC_BREW brew)
  if(CAMLOC_BREW)
    execute_process(COMMAND "${CAMLOC_BREW}" --prefix eigen@3
                    OUTPUT_VARIABLE camloc_eigen_keg
                    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(camloc_eigen_keg AND IS_DIRECTORY "${camloc_eigen_keg}")
      list(APPEND CMAKE_PREFIX_PATH "${camloc_eigen_keg}")
    endif()
  endif()
endif()

# RoboStack's rcutils carries INTERFACE_LINK_LIBRARIES "atomic", which is a
# Linux assumption: macOS has no libatomic, because the atomic builtins live in
# libc++ and compiler-rt. CMake resolves a plain link name against targets
# before it falls back to -l, so an empty interface target absorbs the request
# and the linker never sees -latomic.
if(APPLE AND NOT TARGET atomic)
  add_library(atomic INTERFACE)
endif()

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_path(CAMLOC_STB_INCLUDE_DIR
  NAMES stb_image.h
  HINTS "${camloc_root_parent}/${camloc_root_name}-deps/stb"
  PATH_SUFFIXES stb
  REQUIRED)

find_library(CAMLOC_CORE_LIB cam_loc_core PATHS "${CAMLOC_BUILD_DIR}/src" NO_DEFAULT_PATH REQUIRED)
find_library(CAMLOC_CUDA_LIB cam_loc_cuda PATHS "${CAMLOC_BUILD_DIR}/src/cuda" NO_DEFAULT_PATH REQUIRED)

# ${<pkg>_TARGETS} is every target the message package exports, which includes
# its Python bindings. A C++ node has no use for them, and on macOS they are
# fatal: the *__rosidl_generator_py dylibs resolve Python symbols only inside a
# Python interpreter, so the node links cleanly and then dies at launch with
#   dyld: symbol not found in flat namespace '_PyExc_RuntimeError'
# Linux tolerates it, which is why this survived unnoticed. Keep the C and C++
# typesupport, drop the bindings.
set(CAMLOC_MSG_TARGETS "")
foreach(camloc_msg_target IN LISTS
        visualization_msgs_TARGETS nav_msgs_TARGETS geometry_msgs_TARGETS
        sensor_msgs_TARGETS std_msgs_TARGETS)
  if(NOT camloc_msg_target MATCHES "rosidl_generator_py")
    list(APPEND CAMLOC_MSG_TARGETS "${camloc_msg_target}")
  endif()
endforeach()

# Naming the targets is not enough: rclcpp's own interface pulls in the Python
# bindings of the packages it uses -- rcl_interfaces, builtin_interfaces,
# rosgraph_msgs, statistics_msgs -- so they arrive transitively however
# carefully the list above is written. Walk the imported targets and prune them
# at the source.
function(camloc_drop_py_bindings)
  set(pending ${ARGN})
  set(visited "")
  while(pending)
    list(POP_FRONT pending camloc_target)
    if(NOT TARGET ${camloc_target} OR ${camloc_target} IN_LIST visited)
      continue()
    endif()
    list(APPEND visited ${camloc_target})
    get_target_property(camloc_libs ${camloc_target} INTERFACE_LINK_LIBRARIES)
    if(NOT camloc_libs)
      continue()
    endif()
    set(camloc_kept "")
    foreach(camloc_lib IN LISTS camloc_libs)
      if(camloc_lib MATCHES "rosidl_generator_py")
        continue()
      endif()
      list(APPEND camloc_kept "${camloc_lib}")
      list(APPEND pending "${camloc_lib}")
    endforeach()
    set_target_properties(${camloc_target} PROPERTIES
                          INTERFACE_LINK_LIBRARIES "${camloc_kept}")
  endwhile()
endfunction()

camloc_drop_py_bindings(rclcpp::rclcpp ${CAMLOC_MSG_TARGETS})

add_library(cam_loc_ros_markers src/markers.cc)
target_include_directories(cam_loc_ros_markers
  PUBLIC
    include
    "${CAMLOC_ROOT}/include"
)
# Modern target style (ament_target_dependencies was removed in newer ament_cmake).
target_link_libraries(cam_loc_ros_markers PUBLIC
  Eigen3::Eigen
  ${CAMLOC_CORE_LIB} ${CAMLOC_CUDA_LIB} ${CAMLOC_CUDA_RUNTIME}
  rclcpp::rclcpp
  ${CAMLOC_MSG_TARGETS}
)

add_executable(cam_loc_viz_node src/loc_viz_node.cc src/keyboard_input.cc)
target_include_directories(cam_loc_viz_node PRIVATE
  include
  "${CAMLOC_ROOT}/include"
)
target_include_directories(cam_loc_viz_node SYSTEM PRIVATE "${CAMLOC_STB_INCLUDE_DIR}")
target_link_libraries(cam_loc_viz_node
  cam_loc_ros_markers
  ${CAMLOC_CORE_LIB} ${CAMLOC_CUDA_LIB} ${CAMLOC_CUDA_RUNTIME}
  rclcpp::rclcpp
  ${CAMLOC_MSG_TARGETS}
)

install(TARGETS cam_loc_viz_node cam_loc_ros_markers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
