/* HIP-port forwarding shim: resolves host #include <driver_types.h> to the
 * ElasticFusion CUDA-to-HIP compat header on AMD. Added to the include path only
 * under USE_HIP; on NVIDIA this dir is absent and the real toolkit header wins.
 * Authored with the assistance of Claude (Anthropic). */
#include "../Core/Cuda/cuda_to_hip.h"
