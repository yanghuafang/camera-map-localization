if(CAMLOC_BUILD_CUDA)
  find_program(CAMLOC_NVCC_EXECUTABLE NAMES nvcc PATHS /usr/lib/ccache ENV PATH)
  if(NOT CAMLOC_NVCC_EXECUTABLE)
    find_program(CAMLOC_NVCC_EXECUTABLE NAMES nvcc)
  endif()
endif()

if(CAMLOC_BUILD_CUDA AND CAMLOC_NVCC_EXECUTABLE)
  set(KERNEL_OBJ "${CMAKE_CURRENT_BINARY_DIR}/distance_transform_kernels.o")
  add_custom_command(
    OUTPUT ${KERNEL_OBJ}
    # -Xcompiler forwards to the host compiler nvcc drives, which is what
    # actually parses everything outside the kernels. Without it this was the
    # one translation unit in the tree built without -Wall -Wextra, and the
    # only one no CI job could warn about, since GitHub's runners have no nvcc.
    COMMAND ${CAMLOC_NVCC_EXECUTABLE} -allow-unsupported-compiler -std=c++17
            -Xcompiler=-Wall,-Wextra
            -I${CMAKE_SOURCE_DIR}/src -c
            ${CMAKE_CURRENT_SOURCE_DIR}/distance_transform_kernels.cu -o ${KERNEL_OBJ}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/distance_transform_kernels.cu
            ${CMAKE_CURRENT_SOURCE_DIR}/distance_transform_kernels.h
    COMMENT "Compiling CUDA pose-grid kernels"
    VERBATIM
  )
  add_library(cam_loc_cuda ${KERNEL_OBJ} distance_transform_gpu.cc)
  target_compile_definitions(cam_loc_cuda PUBLIC CAMLOC_CUDA_ENABLED)
  target_include_directories(cam_loc_cuda PUBLIC ${CMAKE_SOURCE_DIR}/include)
  # src/ rather than src/cuda/: the private kernel header is included as
  # "cuda/distance_transform_kernels.h", a path from the source root, which
  # is how Google style spells a project header.
  target_include_directories(cam_loc_cuda PRIVATE ${CMAKE_SOURCE_DIR}/src)
  target_link_libraries(cam_loc_cuda PUBLIC cudart)
  if(EXISTS "/usr/local/cuda/include")
    target_include_directories(cam_loc_cuda PRIVATE /usr/local/cuda/include)
  endif()
  set(CAMLOC_HAS_CUDA TRUE PARENT_SCOPE)
else()
  if(CAMLOC_BUILD_CUDA)
    message(WARNING "nvcc not found; building CPU-only CUDA stub")
  endif()
  add_library(cam_loc_cuda distance_transform_stub.cc)
  target_link_libraries(cam_loc_cuda PUBLIC Eigen3::Eigen)
  target_include_directories(cam_loc_cuda PUBLIC ${CMAKE_SOURCE_DIR}/include)
  # The stub declares the same entry points as the real wrapper, so defining the
  # macro against it compiles every #ifdef CAMLOC_CUDA_ENABLED block in
  # src/core and src/benchmark without needing a GPU. PUBLIC so the definition
  # reaches cam_loc_core, which is where those blocks live.
  if(CAMLOC_CUDA_HOST_ONLY)
    target_compile_definitions(cam_loc_cuda PUBLIC CAMLOC_CUDA_ENABLED)
  endif()
  set(CAMLOC_HAS_CUDA FALSE PARENT_SCOPE)
endif()
