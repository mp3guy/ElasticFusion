cmake_minimum_required(VERSION 3.22)

project(libefusion)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${SUITESPARSE_INCLUDE_DIRS})

file(GLOB srcs *.cpp *.h *.cu *.cuh)
file(GLOB utils_srcs Utils/*.cpp Utils/*.h Utils/*.cu Utils/*.cuh)
file(GLOB shader_srcs Shaders/*.cpp Shaders/*.h Shaders/*.cu Shaders/*.cuh)
file(GLOB cuda Cuda/*.cpp Cuda/*.h Cuda/*.cu Cuda/*.cuh)
file(GLOB containers Cuda/containers/*.cpp Cuda/containers/*.h Cuda/containers/*.cu Cuda/containers/*.cuh)

if(USE_HIP)
  # AMD path: compile the .cu translation units as the HIP language. This
  # replaces the legacy FindCUDA CUDA_COMPILE / CudaDetect / CudaComputeTargetFlags
  # flow (nvcc -gencode); the .cu are added directly to the efusion target.
  enable_language(HIP)

  # Never hardcode the lead arch: a literal here would override
  # -DCMAKE_HIP_ARCHITECTURES and force every follower (gfx1100, gfx1151) to edit
  # this file, churning the curated commit. Default the lead arch only when unset.
  if(NOT DEFINED CMAKE_HIP_ARCHITECTURES OR CMAKE_HIP_ARCHITECTURES STREQUAL "")
    set(CMAKE_HIP_ARCHITECTURES "gfx90a")
  endif()

  # Mark ONLY the .cu translation units LANGUAGE HIP. The glob also captures the
  # .cuh/.h headers; tagging a header LANGUAGE HIP would make CMake try to compile
  # it as a standalone TU (operators.cuh references mat33 from types.cuh and fails
  # in isolation). Headers stay non-compiled.
  set(cuda_tus "")
  foreach(f ${cuda})
    if(f MATCHES "\\.cu$")
      list(APPEND cuda_tus ${f})
    endif()
  endforeach()
  set_source_files_properties(${cuda_tus} PROPERTIES LANGUAGE HIP)

  # Force-include the CUDA-to-HIP compat header on every HIP TU so its
  # __CUDACC__ / mask / alias defines precede any other include regardless of
  # per-file include order.
  set(CMAKE_HIP_FLAGS
      "${CMAKE_HIP_FLAGS} -fPIC -include ${CMAKE_CURRENT_SOURCE_DIR}/Cuda/cuda_to_hip.h")
  include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/../hip_compat")

  add_library(efusion SHARED
              ${srcs}
              ${utils_srcs}
              ${shader_srcs}
              ${cuda}
              ${containers}
  )

  set_target_properties(efusion PROPERTIES HIP_ARCHITECTURES "${CMAKE_HIP_ARCHITECTURES}")

  target_link_libraries(efusion
                        ${Eigen_LIBRARIES}
                        ${Pangolin_LIBRARIES}
                        ${SUITESPARSE_LIBRARIES}
  )
else()
  set(CUDA_ARCH_BIN "" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
  set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

  include("${CMAKE_MODULE_PATH}/CudaDetect.cmake")
  detect_installed_gpus(CUDA_NVCC_ARCHS)
  foreach(NVCC_ARCH IN LISTS CUDA_NVCC_ARCHS)
      list(APPEND CUDA_ARCH_BIN "${NVCC_ARCH} ")
  endforeach(NVCC_ARCH)

  include("${CMAKE_MODULE_PATH}/CudaComputeTargetFlags.cmake")
  APPEND_TARGET_ARCH_FLAGS()

  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;--expt-relaxed-constexpr;--disable-warnings")

  CUDA_COMPILE(cuda_objs ${cuda})

  add_library(efusion SHARED
              ${srcs}
              ${utils_srcs}
              ${shader_srcs}
              ${cuda}
              ${cuda_objs}
              ${containers}
  )

  target_link_libraries(efusion
                        ${Eigen_LIBRARIES}
                        ${Pangolin_LIBRARIES}
                        ${CUDA_LIBRARIES}
                        ${SUITESPARSE_LIBRARIES}
  )
endif()
