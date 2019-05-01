#include "convenience.cuh"
#include "estimate.cuh"

template <int D>
__inline__ __device__ void warpReduceSum(Eigen::Matrix<float, D, 1>& val) {
  for (int offset = warpSize / 2; offset > 0; offset /= 2) {
#pragma unroll
    for (int i = 0; i < D; i++) {
      val[i] += __shfl_down_sync(0xFFFFFFFF, val[i], offset);
    }
  }
}

template <int D>
__inline__ __device__ void blockReduceSum(Eigen::Matrix<float, D, 1>& val) {
  // Allocate shared memory in two steps otherwise NVCC complains about Eigen's
  // non-empty constructor
  static __shared__ unsigned char sharedMem[32 * sizeof(Eigen::Matrix<float, D, 1>)];

  Eigen::Matrix<float, D, 1>(&shared)[32] =
      reinterpret_cast<Eigen::Matrix<float, D, 1>(&)[32]>(sharedMem);

  int lane = threadIdx.x % warpSize;

  int wid = threadIdx.x / warpSize;

  warpReduceSum(val);

  // write reduced value to shared memory
  if (lane == 0) {
    shared[wid] = val;
  }
  __syncthreads();

  // ensure we only grab a value from shared memory if that warp existed
  val = (threadIdx.x < blockDim.x / warpSize) ? shared[lane] : Eigen::Matrix<float, D, 1>::Zero();

  if (wid == 0) {
    warpReduceSum(val);
  }
}

template <int D>
__global__ void reduceSum(Eigen::Matrix<float, D, 1>* in, Eigen::Matrix<float, D, 1>* out, int N) {
  Eigen::Matrix<float, D, 1> sum = Eigen::Matrix<float, D, 1>::Zero();

  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x) {
    sum += in[i];
  }

  blockReduceSum(sum);

  if (threadIdx.x == 0) {
    out[blockIdx.x] = sum;
  }
}

struct Reduction {
  Eigen::Matrix3f R_prev_curr;
  Eigen::Vector3f t_prev_curr;

  CameraModel intr;

  PtrStep<float> vmap_curr;
  PtrStep<float> nmap_curr;

  PtrStep<float> vmap_prev;
  PtrStep<float> nmap_prev;

  float dist_thresh;
  float angle_thresh;

  int cols;
  int rows;
  int N;

  Eigen::Matrix<float, 29, 1>* out;

  // And now for some template metaprogramming magic
  template <int outer, int inner, int end>
  struct SquareUpperTriangularProduct {
    __device__ __forceinline__ static void apply(
        Eigen::Matrix<float, 29, 1>& values,
        const float (&rows)[end + 1]) {
      values[((end + 1) * outer) + inner - (outer * (outer + 1) / 2)] = rows[outer] * rows[inner];

      SquareUpperTriangularProduct<outer, inner + 1, end>::apply(values, rows);
    }
  };

  // Inner loop base
  template <int outer, int end>
  struct SquareUpperTriangularProduct<outer, end, end> {
    __device__ __forceinline__ static void apply(
        Eigen::Matrix<float, 29, 1>& values,
        const float (&rows)[end + 1]) {
      values[((end + 1) * outer) + end - (outer * (outer + 1) / 2)] = rows[outer] * rows[end];

      SquareUpperTriangularProduct<outer + 1, outer + 1, end>::apply(values, rows);
    }
  };

  // Outer loop base
  template <int end>
  struct SquareUpperTriangularProduct<end, end, end> {
    __device__ __forceinline__ static void apply(
        Eigen::Matrix<float, 29, 1>& values,
        const float (&rows)[end + 1]) {
      values[((end + 1) * end) + end - (end * (end + 1) / 2)] = rows[end] * rows[end];
    }
  };

  __device__ __forceinline__ void operator()() const {
    Eigen::Matrix<float, 29, 1> sum = Eigen::Matrix<float, 29, 1>::Zero();

    SquareUpperTriangularProduct<0, 0, 6> sutp;

    Eigen::Matrix<float, 29, 1> values;

    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x) {
      const int y = i / cols;
      const int x = i - (y * cols);

      const Eigen::Vector3f v_curr(
          vmap_curr.ptr(y)[x], vmap_curr.ptr(y + rows)[x], vmap_curr.ptr(y + 2 * rows)[x]);

      const Eigen::Vector3f v_curr_in_prev = R_prev_curr * v_curr + t_prev_curr;

      const Eigen::Matrix<int, 2, 1> p_curr_in_prev(
          __float2int_rn(v_curr_in_prev(0) * intr.fx / v_curr_in_prev(2) + intr.cx),
          __float2int_rn(v_curr_in_prev(1) * intr.fy / v_curr_in_prev(2) + intr.cy));

      float row[7] = {0, 0, 0, 0, 0, 0, 0};

      values[28] = 0;

      if (p_curr_in_prev(0) >= 0 && p_curr_in_prev(1) >= 0 && p_curr_in_prev(0) < cols &&
          p_curr_in_prev(1) < rows && v_curr(2) > 0 && v_curr_in_prev(2) > 0) {
        const Eigen::Vector3f v_prev(
            vmap_prev.ptr(p_curr_in_prev(1))[p_curr_in_prev(0)],
            vmap_prev.ptr(p_curr_in_prev(1) + rows)[p_curr_in_prev(0)],
            vmap_prev.ptr(p_curr_in_prev(1) + 2 * rows)[p_curr_in_prev(0)]);

        const Eigen::Vector3f n_curr(
            nmap_curr.ptr(y)[x], nmap_curr.ptr(y + rows)[x], nmap_curr.ptr(y + 2 * rows)[x]);

        const Eigen::Vector3f n_curr_in_prev = R_prev_curr * n_curr;

        const Eigen::Vector3f n_prev(
            nmap_prev.ptr(p_curr_in_prev(1))[p_curr_in_prev(0)],
            nmap_prev.ptr(p_curr_in_prev(1) + rows)[p_curr_in_prev(0)],
            nmap_prev.ptr(p_curr_in_prev(1) + 2 * rows)[p_curr_in_prev(0)]);

        if (n_curr_in_prev.cross(n_prev).norm() < angle_thresh &&
            (v_prev - v_curr_in_prev).norm() < dist_thresh && !isnan(n_curr(0)) &&
            !isnan(n_prev(0))) {
          *(Eigen::Vector3f*)&row[0] = n_prev;
          *(Eigen::Vector3f*)&row[3] = v_curr_in_prev.cross(n_prev);
          row[6] = n_prev.dot(v_prev - v_curr_in_prev);

          values[28] = 1;

          sutp.apply(values, row);

          sum += values;
        }
      }
    }

    blockReduceSum(sum);

    if (threadIdx.x == 0) {
      out[blockIdx.x] = sum;
    }
  }
};

__global__ void estimateKernel(const Reduction reduction) {
  reduction();
}

void estimateStep(
    const Eigen::Matrix3f& R_prev_curr,
    const Eigen::Vector3f& t_prev_curr,
    const DeviceArray2D<float>& vmap_curr,
    const DeviceArray2D<float>& nmap_curr,
    const CameraModel& intr,
    const DeviceArray2D<float>& vmap_prev,
    const DeviceArray2D<float>& nmap_prev,
    float dist_thresh,
    float angle_thresh,
    DeviceArray<Eigen::Matrix<float, 29, 1>>& sum,
    DeviceArray<Eigen::Matrix<float, 29, 1>>& out,
    float* matrixA_host,
    float* vectorB_host,
    float* residual_inliers) {
  int cols = vmap_curr.cols();
  int rows = vmap_curr.rows() / 3;

  Reduction reduction;

  reduction.R_prev_curr = R_prev_curr;
  reduction.t_prev_curr = t_prev_curr;

  reduction.vmap_curr = vmap_curr;
  reduction.nmap_curr = nmap_curr;

  reduction.intr = intr;

  reduction.vmap_prev = vmap_prev;
  reduction.nmap_prev = nmap_prev;

  reduction.dist_thresh = dist_thresh;
  reduction.angle_thresh = angle_thresh;

  reduction.cols = cols;
  reduction.rows = rows;

  reduction.N = cols * rows;
  reduction.out = sum;

  estimateKernel<<<reduceBlocks, reduceThreads>>>(reduction);

  reduceSum<29><<<1, MAX_THREADS>>>(sum, out, reduceBlocks);

  cudaSafeCall(cudaGetLastError());
  cudaSafeCall(cudaDeviceSynchronize());

  float host_data[29];
  out.download((Eigen::Matrix<float, 29, 1>*)&host_data[0]);

  int shift = 0;
  for (int i = 0; i < 6; ++i) // rows
  {
    for (int j = i; j < 7; ++j) // cols + b
    {
      float value = host_data[shift++];
      if (j == 6) // vector b
        vectorB_host[i] = value;
      else
        matrixA_host[j * 6 + i] = matrixA_host[i * 6 + j] = value;
    }
  }

  residual_inliers[0] = host_data[27];
  residual_inliers[1] = host_data[28];
}
