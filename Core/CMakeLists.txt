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
