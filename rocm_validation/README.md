# ROCm/HIP validation for ElasticFusion

This directory holds the GPU-correctness validation for the ElasticFusion
CUDA-to-HIP port (Linux gfx90a, MI250X, ROCm 7.2.x). It was
authored with the assistance of Claude (Anthropic).

## Why a custom harness (no GL replay)

ElasticFusion ships no gtest/ctest suite; it is normally validated by replaying
an RGB-D `.klg` log through the full application. That path is **infeasible on a
headless MI250X**, and the reason is the key porting finding for this project:

ElasticFusion's tracking inputs reach the CUDA/HIP kernels **only** through
OpenGL: `RGBDOdometry` maps GL textures (the GLSL-splatted surfel prediction and
the filtered depth / RGB) to a `cudaArray_t` via `cudaGraphicsGLRegisterImage` +
`cudaGraphicsMapResources` + `cudaGraphicsSubResourceGetMappedArray`, then feeds
that array to the kernels. There is no CUDA-only ingest path.

On this node, GL/HIP interop cannot be established (see `gl_interop_probe.cpp`):

- The gfx90a is a **compute-only** chip. Mesa's hardware GL driver (radeonsi)
  refuses to create a context on it: `radeonsi: error: can't create a graphics
  context on a compute chip`. So there is no GPU-resident GL context to share.
- There is no Vulkan ICD installed, so zink (GL-over-Vulkan) cannot initialize
  (`VK_ERROR_INCOMPATIBLE_DRIVER`).
- Software GL (llvmpipe) renders fine (GL 4.5) but lives in host memory, so
  `hipGLGetDevices` returns `hipErrorNoDevice` and `hipGraphicsGLRegisterImage`
  returns `hipErrorUnknown` -- HIP cannot import a CPU-rendered texture onto the
  gfx90a device.

The HIP GL-interop API itself is fully present in ROCm 7.2.x (`hip_gl_interop.h`,
all symbols in hipify's map); the wall is purely the absence of a GPU-backed GL
stack on a headless compute accelerator.

So GPU validation here exercises the **ported tracking kernels directly** via a
device-array harness (no GL), which is exactly the device code the SLAM tracker
drives once a frame's data is on the GPU.

## `kernel_harness.cpp`

Feeds fixed `DeviceArray` inputs into the ported entry points and checks results
on real gfx90a:

- `createVMap` -- vmap equals a CPU back-projection of the depth image (exact).
- `createNMap` -- a dense field of unit-length normals (cross-product path).
- `tranformMaps` -- identity pose leaves the vmap bit-stable.
- `resizeVMap` -- correct half-resolution averaging.
- `icpStep` (SE3) -- the heaviest wave64 reduction: JtJ symmetric, positive
  diagonal, residual 0 at identity, and **identical across two runs**.
- `so3Step` (SO3) -- wave64 reduction, **identical across two runs**.
- `computeRgbResidual` (int2) -- the int reduction count and sum-of-squares
  **exactly equal** a CPU recount of the device-written correspondences. Because
  integer addition is order-independent, this is a hard arithmetic proof that the
  wave64 reduction tree sums every one of the 64 lanes with none lost or
  double-counted. Also exercises the Sobel `__constant__` + `cudaMemcpyToSymbol`.
- `rgbStep` (SE3 photometric) -- wave64 reduction, symmetric, deterministic.

The two-run bit-identity checks fingerprint the wave64 reduction
non-determinism class; combined with the exact integer cross-check they prove
the `__shfl_down_sync` reduction is correct on the 64-lane wavefront.

## Build and run

Build the HIP `efusion` library first (from the repo root):

```
cmake -S . -B build-hip -DUSE_HIP=ON -DCMAKE_HIP_ARCHITECTURES=gfx90a \
      -DCMAKE_HIP_COMPILER=/opt/rocm/llvm/bin/clang++ -DCMAKE_BUILD_TYPE=Release
make -C build-hip efusion -j16
```

Then build and run the harness against the ported kernel TUs:

```
SRC=..
HIPCC=/opt/rocm/bin/hipcc
$HIPCC -std=c++17 --offload-arch=gfx90a -O2 -fPIC -DUSE_HIP \
  -include $SRC/Core/Cuda/cuda_to_hip.h -I $SRC/Core/Cuda -I $SRC/hip_compat \
  -c $SRC/Core/Cuda/cudafuncs.cu -o cudafuncs.o
$HIPCC -std=c++17 --offload-arch=gfx90a -O2 -fPIC -DUSE_HIP \
  -include $SRC/Core/Cuda/cuda_to_hip.h -I $SRC/Core/Cuda -I $SRC/hip_compat \
  -c $SRC/Core/Cuda/reduce.cu -o reduce.o
g++ -std=c++17 -O2 -fPIC -DUSE_HIP -D__HIP_PLATFORM_AMD__ -I /opt/rocm/include \
  -I $SRC/hip_compat -I $SRC/Core/Cuda -I $SRC/Core/Cuda/containers \
  -c $SRC/Core/Cuda/containers/device_memory.cpp -o device_memory.o
$HIPCC -std=c++17 -O2 -DUSE_HIP -D__HIP_PLATFORM_AMD__ \
  -I $SRC/Core/Cuda -I $SRC/Core/Cuda/containers -I $SRC/hip_compat \
  -c kernel_harness.cpp -o kernel_harness.o
$HIPCC -std=c++17 --offload-arch=gfx90a \
  kernel_harness.o cudafuncs.o reduce.o device_memory.o -o kernel_harness
HIP_VISIBLE_DEVICES=2 ./kernel_harness
```

Expected: `HARNESS PASSED (0 failures)`.

`gl_interop_probe.cpp` (build: `hipcc -std=c++17 gl_interop_probe.cpp -lEGL -lGL`)
reproduces the GL/HIP interop infeasibility finding above; it prints `RESULT:
INTEROP_*` and exits non-zero on this hardware.
