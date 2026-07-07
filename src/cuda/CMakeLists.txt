if(CAMLOC_BUILD_CUDA)
  find_program(CAMLOC_NVCC_EXECUTABLE NAMES nvcc PATHS /usr/lib/ccache ENV PATH)
  if(NOT CAMLOC_NVCC_EXECUTABLE)
    find_program(CAMLOC_NVCC_EXECUTABLE NAMES nvcc)
  endif()

  if(CAMLOC_NVCC_EXECUTABLE)
    set(KERNEL_OBJ "${CMAKE_CURRENT_BINARY_DIR}/distance_transform_kernels.o")
    add_custom_command(
      OUTPUT ${KERNEL_OBJ}
      COMMAND ${CAMLOC_NVCC_EXECUTABLE} -allow-unsupported-compiler -std=c++17
              -I${CMAKE_CURRENT_SOURCE_DIR} -c
              ${CMAKE_CURRENT_SOURCE_DIR}/distance_transform_kernels.cu -o ${KERNEL_OBJ}
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/distance_transform_kernels.cu
              ${CMAKE_CURRENT_SOURCE_DIR}/distance_transform_kernels.cuh
      COMMENT "Compiling CUDA pose-grid kernels"
      VERBATIM
    )
    add_library(cam_loc_cuda ${KERNEL_OBJ} distance_transform_gpu.cpp)
    target_compile_definitions(cam_loc_cuda PUBLIC CAMLOC_CUDA_ENABLED)
    target_include_directories(cam_loc_cuda PUBLIC ${CMAKE_SOURCE_DIR}/include)
    target_include_directories(cam_loc_cuda PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(cam_loc_cuda PUBLIC cudart)
    if(EXISTS "/usr/local/cuda/include")
      target_include_directories(cam_loc_cuda PRIVATE /usr/local/cuda/include)
    endif()
    set(CAMLOC_HAS_CUDA TRUE PARENT_SCOPE)
  else()
    message(WARNING "nvcc not found; building CPU-only CUDA stub")
    add_library(cam_loc_cuda distance_transform_stub.cpp)
    target_link_libraries(cam_loc_cuda PUBLIC Eigen3::Eigen)
    target_include_directories(cam_loc_cuda PUBLIC ${CMAKE_SOURCE_DIR}/include)
    set(CAMLOC_HAS_CUDA FALSE PARENT_SCOPE)
  endif()
else()
  add_library(cam_loc_cuda distance_transform_stub.cpp)
  target_link_libraries(cam_loc_cuda PUBLIC Eigen3::Eigen)
  target_include_directories(cam_loc_cuda PUBLIC ${CMAKE_SOURCE_DIR}/include)
  set(CAMLOC_HAS_CUDA FALSE PARENT_SCOPE)
endif()
