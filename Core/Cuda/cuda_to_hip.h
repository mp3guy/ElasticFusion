/*
 * This file is part of the ElasticFusion ROCm/HIP port.
 *
 * CUDA-to-HIP compatibility shim (colmap-model Strategy A). On ROCm it includes
 * the HIP runtime and aliases only the CUDA symbols ElasticFusion actually uses
 * to their 1:1 HIP equivalents; on NVIDIA it is a no-op include of the CUDA
 * runtime. This is the only file that knows about HIP. It is force-included on
 * every HIP translation unit (CMAKE_HIP_FLAGS -include ...) so the defines below
 * precede any other include regardless of per-file include order.
 *
 * Authored with the assistance of Claude (Anthropic).
 */
#ifndef ELASTICFUSION_CUDA_TO_HIP_H_
#define ELASTICFUSION_CUDA_TO_HIP_H_

#if defined(USE_HIP) || defined(__HIP_PLATFORM_AMD__)

// Only the HIP COMPILER (hipcc/clang -x hip) defines __HIPCC__, and only on the
// .cu translation units. The host .cpp (GPUTexture, RGBDOdometry, the container
// layer) are built by the CXX compiler and need the cuda*->hip* symbol aliases
// below but MUST NOT see __CUDACC__ / EIGEN_NO_CUDA: defining __CUDACC__ in host
// code makes Pangolin's headers hide their Eigen declarations (their own
// `#ifndef __CUDACC__` guards) and excludes types.cuh's Eigen ctor of mat33,
// breaking the host build. So gate the device-compiler defines on __HIPCC__.
#if defined(__HIPCC__)
// hipcc defines __HIPCC__ in both passes but NOT __CUDACC__. ElasticFusion's
// container layer (kernel_containers.hpp) gates GPU_HOST_DEVICE__ on __CUDACC__,
// and types.cuh gates the host-only Eigen ctor of mat33 on !__CUDACC__. Define
// __CUDACC__ on the HIP TU so __host__ __device__ decoration is applied and the
// Eigen ctor is excluded from the device TU. Safe here because ElasticFusion
// pulls in NO rocThrust/rocPRIM (which key their backend on __CUDACC__).
#if !defined(__CUDACC__)
#define __CUDACC__ 1
#endif
// We do NOT define __CUDA_ARCH__: types.cuh excludes Eigen from the device pass
// via __CUDACC__, so Eigen never reaches device code. Define EIGEN_NO_CUDA
// defensively in case an Eigen header is dragged in transitively, so Eigen takes
// its native HIP device path rather than #include <math_constants.h>.
#ifndef EIGEN_NO_CUDA
#define EIGEN_NO_CUDA
#endif
#endif  // __HIPCC__

#include <hip/hip_runtime.h>
#include <hip/hip_gl_interop.h>

// Defining __CUDACC__ (above) makes HIP's __device__ memset/memcpy (declared in
// amd_device_functions.h) participate in unqualified name lookup; pull in the
// host <cstring> so the global ::memset/::memcpy remain viable candidates in the
// host-only helpers (e.g. initTextureObjectFromArray's memset of the resource
// descriptor). Harmless on CUDA, which is the #else branch.
#include <cstring>

// Full-warp lane mask for __shfl_*_sync. HIP's *_sync builtins static_assert
// sizeof(mask)==8, so the CUDA 32-bit 0xFFFFFFFF literal fails to COMPILE on
// HIP; this is also the wave64-correct mask (all 64 lanes participate). The
// reductions in reduce.cu reference CUDA_WARP_FULL_MASK instead of a literal.
#define CUDA_WARP_FULL_MASK 0xffffffffffffffffULL

// --- runtime / memory ---
#define cudaError_t                       hipError_t
#define cudaSuccess                       hipSuccess
#define cudaGetLastError                  hipGetLastError
#define cudaGetErrorString                hipGetErrorString
#define cudaDeviceSynchronize             hipDeviceSynchronize
#define cudaMalloc                        hipMalloc
#define cudaFree                          hipFree
#define cudaMallocPitch                   hipMallocPitch
#define cudaMemcpy                        hipMemcpy
#define cudaMemcpy2D                      hipMemcpy2D
#define cudaMemcpy2DFromArray             hipMemcpy2DFromArray
#define cudaMemcpyToSymbol                hipMemcpyToSymbol
#define cudaMemcpyHostToDevice            hipMemcpyHostToDevice
#define cudaMemcpyDeviceToHost            hipMemcpyDeviceToHost
#define cudaMemcpyDeviceToDevice          hipMemcpyDeviceToDevice

// --- arrays ---
#define cudaArray_t                       hipArray_t

// --- texture objects (all point-filtered, array-backed, element-read) ---
#define cudaTextureObject_t               hipTextureObject_t
#define cudaResourceDesc                  hipResourceDesc
#define cudaTextureDesc                   hipTextureDesc
#define cudaCreateTextureObject           hipCreateTextureObject
#define cudaDestroyTextureObject          hipDestroyTextureObject
#define cudaResourceTypeArray             hipResourceTypeArray
#define cudaFilterModePoint               hipFilterModePoint
#define cudaAddressModeClamp              hipAddressModeClamp
#define cudaReadModeElementType           hipReadModeElementType

// --- OpenGL interop ---
#define cudaGraphicsResource              hipGraphicsResource
#define cudaGraphicsGLRegisterImage       hipGraphicsGLRegisterImage
#define cudaGraphicsMapResources          hipGraphicsMapResources
#define cudaGraphicsSubResourceGetMappedArray hipGraphicsSubResourceGetMappedArray
#define cudaGraphicsUnmapResources        hipGraphicsUnmapResources
#define cudaGraphicsUnregisterResource    hipGraphicsUnregisterResource
#define cudaGraphicsRegisterFlagsReadOnly hipGraphicsRegisterFlagsReadOnly

#else  // CUDA path: no-op, plain CUDA spelling everywhere.

#include <cuda_runtime.h>
#define CUDA_WARP_FULL_MASK 0xFFFFFFFFu

#endif

#endif /* ELASTICFUSION_CUDA_TO_HIP_H_ */
