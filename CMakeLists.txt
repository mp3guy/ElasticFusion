cmake_minimum_required(VERSION 3.22)

project(ElasticFusion)

option(USE_HIP "Build the CUDA tracking code with HIP for AMD GPUs" OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
set(CMAKE_CXX_STANDARD 17)

set(Pangolin_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third-party/Pangolin/build/src" CACHE PATH "Pangolin build directory")
set(OPENNI2_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third-party/OpenNI2/Include" CACHE PATH "OpenNI2 Include directory")
set(OPENNI2_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/third-party/OpenNI2/Bin/x64-Release/libOpenNI2.so" CACHE PATH "OpenNI2 library")
set(Sophus_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third-party/Sophus" CACHE PATH "Sophus Include directory")
set(efusion_SHADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Core/Shaders" CACHE PATH "Where the shaders live")

find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(ZLIB REQUIRED)
find_package(JPEG REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
if(NOT USE_HIP)
  find_package(CUDA REQUIRED)
endif()
find_package(OpenNI2 REQUIRED)
find_package(SuiteSparse REQUIRED)

include_directories(${JPEG_INCLUDE_DIR})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIR})

# On AMD the host .cpp keep their <cuda_gl_interop.h>/<cuda_runtime_api.h>/
# <driver_types.h>/<vector_types.h> includes; the hip_compat/ shims resolve those
# toolkit names to the CUDA-to-HIP compat header. This dir is added only under
# USE_HIP, so the NVIDIA build is unchanged. The host C++ files that
# touch the CUDA runtime (GPUTexture, RGBDOdometry, the container layer) compile
# with the CXX compiler, so they need USE_HIP / __HIP_PLATFORM_AMD__ defined and
# the ROCm include path to pick up the HIP host runtime.
if(USE_HIP)
  if(NOT DEFINED ROCM_PATH)
    if(DEFINED ENV{ROCM_PATH})
      set(ROCM_PATH $ENV{ROCM_PATH})
    else()
      set(ROCM_PATH "/opt/rocm")
    endif()
  endif()
  include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/hip_compat")
  include_directories("${ROCM_PATH}/include")
  add_definitions(-DUSE_HIP -D__HIP_PLATFORM_AMD__)
endif()

if(WITH_REALSENSE)
  include_directories(${REALSENSE_INCLUDE_DIR})
  add_definitions(-DWITH_REALSENSE)
  set(EXTRA_LIBS ${EXTRA_LIBS} ${REALSENSE_LIBRARY})
endif()

file(GLOB srcs *.cpp *.h *.cu *.cuh)
file(GLOB tools_srcs Tools/*.cpp Tools/*.h Tools/*.cu Tools/*.cuh)

add_definitions(-Dlinux=1)
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0)
add_definitions(-DEIGEN_MAX_STATIC_ALIGN_BYTES=0)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O3 -msse2 -msse3 -Wall -DSHADER_DIR=${efusion_SHADER_DIR}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -DSHADER_DIR=${efusion_SHADER_DIR}")

add_subdirectory(Core)

add_executable(ElasticFusion 
               ${srcs}
               ${tools_srcs}
)

target_link_libraries(ElasticFusion
                      efusion
                      ${ZLIB_LIBRARY}
                      ${JPEG_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${EXTRA_LIBS}
                      ${OPENNI2_LIBRARY}
                      ${SUITESPARSE_LIBRARIES}
                      ${BLAS_LIBRARIES}
                      ${LAPACK_LIBRARIES}
)
