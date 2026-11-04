/* HIP-port forwarding shim: HIP has no toolkit-named <vector_types.h>; float3/
 * float4/short2/etc. and make_floatN come from <hip/hip_runtime.h>, which the
 * compat header includes. Added to the include path only under USE_HIP; on
 * NVIDIA this dir is absent and the real toolkit header wins.
 * Authored with the assistance of Claude (Anthropic). */
#include "../Core/Cuda/cuda_to_hip.h"
