cmake_minimum_required(VERSION 2.6.0)

project(GPUTest)

find_package(Pangolin 0.1 REQUIRED)
find_package(CUDA REQUIRED)


if(TARGET efusion)
    set(efusion_LIBRARY efusion)
    set(efusion_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}../../Core/src)
else()
  find_package(efusion REQUIRED)
endif()

include_directories(/usr/include/suitesparse)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${efusion_INCLUDE_DIR})

file(GLOB srcs *.cpp)

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11")
#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -g -Wall")

add_executable(GPUTest 
               ${srcs}
)

target_link_libraries(GPUTest
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${efusion_LIBRARY}
)

