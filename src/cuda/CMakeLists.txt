# No nvcc build yet: this is the CPU stub, which declares the same entry points
# the GPU wrapper will and reports IsAvailable() false, so every caller takes
# its fallback at run time.
add_library(cam_loc_cuda distance_transform_stub.cc)
target_link_libraries(cam_loc_cuda PUBLIC Eigen3::Eigen)
target_include_directories(cam_loc_cuda PUBLIC ${CMAKE_SOURCE_DIR}/include)
# The stub declares the same entry points as the real wrapper, so defining the
# macro against it compiles every #ifdef CAMLOC_CUDA_ENABLED block in src/core
# without needing a GPU. PUBLIC so the definition reaches cam_loc_core, which is
# where those blocks live.
if(CAMLOC_CUDA_HOST_ONLY)
  target_compile_definitions(cam_loc_cuda PUBLIC CAMLOC_CUDA_ENABLED)
endif()
set(CAMLOC_HAS_CUDA FALSE PARENT_SCOPE)
