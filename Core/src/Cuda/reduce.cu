/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#include "cudafuncs.cuh"
#include "convenience.cuh"
#include "operators.cuh"

__inline__  __device__ JtJJtrSE3 warpReduceSum(JtJJtrSE3 val)
{
    for(int offset = warpSize / 2; offset > 0; offset /= 2)
    {
        val.aa += __shfl_down_sync(0xFFFFFFFF, val.aa, offset);
        val.ab += __shfl_down_sync(0xFFFFFFFF, val.ab, offset);
        val.ac += __shfl_down_sync(0xFFFFFFFF, val.ac, offset);
        val.ad += __shfl_down_sync(0xFFFFFFFF, val.ad, offset);
        val.ae += __shfl_down_sync(0xFFFFFFFF, val.ae, offset);
        val.af += __shfl_down_sync(0xFFFFFFFF, val.af, offset);
        val.ag += __shfl_down_sync(0xFFFFFFFF, val.ag, offset);

        val.bb += __shfl_down_sync(0xFFFFFFFF, val.bb, offset);
        val.bc += __shfl_down_sync(0xFFFFFFFF, val.bc, offset);
        val.bd += __shfl_down_sync(0xFFFFFFFF, val.bd, offset);
        val.be += __shfl_down_sync(0xFFFFFFFF, val.be, offset);
        val.bf += __shfl_down_sync(0xFFFFFFFF, val.bf, offset);
        val.bg += __shfl_down_sync(0xFFFFFFFF, val.bg, offset);

        val.cc += __shfl_down_sync(0xFFFFFFFF, val.cc, offset);
        val.cd += __shfl_down_sync(0xFFFFFFFF, val.cd, offset);
        val.ce += __shfl_down_sync(0xFFFFFFFF, val.ce, offset);
        val.cf += __shfl_down_sync(0xFFFFFFFF, val.cf, offset);
        val.cg += __shfl_down_sync(0xFFFFFFFF, val.cg, offset);

        val.dd += __shfl_down_sync(0xFFFFFFFF, val.dd, offset);
        val.de += __shfl_down_sync(0xFFFFFFFF, val.de, offset);
        val.df += __shfl_down_sync(0xFFFFFFFF, val.df, offset);
        val.dg += __shfl_down_sync(0xFFFFFFFF, val.dg, offset);

        val.ee += __shfl_down_sync(0xFFFFFFFF, val.ee, offset);
        val.ef += __shfl_down_sync(0xFFFFFFFF, val.ef, offset);
        val.eg += __shfl_down_sync(0xFFFFFFFF, val.eg, offset);

        val.ff += __shfl_down_sync(0xFFFFFFFF, val.ff, offset);
        val.fg += __shfl_down_sync(0xFFFFFFFF, val.fg, offset);

        val.residual += __shfl_down_sync(0xFFFFFFFF, val.residual, offset);
        val.inliers += __shfl_down_sync(0xFFFFFFFF, val.inliers, offset);
    }

    return val;
}

__inline__  __device__ JtJJtrSE3 blockReduceSum(JtJJtrSE3 val)
{
    static __shared__ JtJJtrSE3 shared[32];

    int lane = threadIdx.x % warpSize;

    int wid = threadIdx.x / warpSize;

    val = warpReduceSum(val);

    //write reduced value to shared memory
    if(lane == 0)
    {
        shared[wid] = val;
    }
    __syncthreads();

    const JtJJtrSE3 zero = {0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0};

    //ensure we only grab a value from shared memory if that warp existed
    val = (threadIdx.x < blockDim.x / warpSize) ? shared[lane] : zero;

    if(wid == 0)
    {
        val = warpReduceSum(val);
    }

    return val;
}

__global__ void reduceSum(JtJJtrSE3 * in, JtJJtrSE3 * out, int N)
{
    JtJJtrSE3 sum = {0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0};

    for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x)
    {
        sum.add(in[i]);
    }

    sum = blockReduceSum(sum);

    if(threadIdx.x == 0)
    {
        out[blockIdx.x] = sum;
    }
}

__inline__  __device__ JtJJtrSO3 warpReduceSum(JtJJtrSO3 val)
{
    for(int offset = warpSize / 2; offset > 0; offset /= 2)
    {
        val.aa += __shfl_down_sync(0xFFFFFFFF, val.aa, offset);
        val.ab += __shfl_down_sync(0xFFFFFFFF, val.ab, offset);
        val.ac += __shfl_down_sync(0xFFFFFFFF, val.ac, offset);
        val.ad += __shfl_down_sync(0xFFFFFFFF, val.ad, offset);

        val.bb += __shfl_down_sync(0xFFFFFFFF, val.bb, offset);
        val.bc += __shfl_down_sync(0xFFFFFFFF, val.bc, offset);
        val.bd += __shfl_down_sync(0xFFFFFFFF, val.bd, offset);

        val.cc += __shfl_down_sync(0xFFFFFFFF, val.cc, offset);
        val.cd += __shfl_down_sync(0xFFFFFFFF, val.cd, offset);

        val.residual += __shfl_down_sync(0xFFFFFFFF, val.residual, offset);
        val.inliers += __shfl_down_sync(0xFFFFFFFF, val.inliers, offset);
    }

    return val;
}

__inline__  __device__ JtJJtrSO3 blockReduceSum(JtJJtrSO3 val)
{
    static __shared__ JtJJtrSO3 shared[32];

    int lane = threadIdx.x % warpSize;

    int wid = threadIdx.x / warpSize;

    val = warpReduceSum(val);

    //write reduced value to shared memory
    if(lane == 0)
    {
        shared[wid] = val;
    }
    __syncthreads();

    const JtJJtrSO3 zero = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //ensure we only grab a value from shared memory if that warp existed
    val = (threadIdx.x < blockDim.x / warpSize) ? shared[lane] : zero;

    if(wid == 0)
    {
        val = warpReduceSum(val);
    }

    return val;
}

__global__ void reduceSum(JtJJtrSO3 * in, JtJJtrSO3 * out, int N)
{
    JtJJtrSO3 sum = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x)
    {
        sum.add(in[i]);
    }

    sum = blockReduceSum(sum);

    if(threadIdx.x == 0)
    {
        out[blockIdx.x] = sum;
    }
}

struct ICPReduction
{
    mat33 Rcurr;
    float3 tcurr;

    PtrStep<float> vmap_curr;
    PtrStep<float> nmap_curr;

    mat33 Rprev_inv;
    float3 tprev;

    CameraModel intr;

    PtrStep<float> vmap_g_prev;
    PtrStep<float> nmap_g_prev;

    float distThres;
    float angleThres;

    int cols;
    int rows;
    int N;

    JtJJtrSE3 * out;

    __device__ __forceinline__ bool
    search (int & x, int & y, float3& n, float3& d, float3& s) const
    {
        float3 vcurr;
        vcurr.x = vmap_curr.ptr (y       )[x];
        vcurr.y = vmap_curr.ptr (y + rows)[x];
        vcurr.z = vmap_curr.ptr (y + 2 * rows)[x];

        float3 vcurr_g = Rcurr * vcurr + tcurr;
        float3 vcurr_cp = Rprev_inv * (vcurr_g - tprev);

        int2 ukr;
        ukr.x = __float2int_rn (vcurr_cp.x * intr.fx / vcurr_cp.z + intr.cx);
        ukr.y = __float2int_rn (vcurr_cp.y * intr.fy / vcurr_cp.z + intr.cy);

        if(ukr.x < 0 || ukr.y < 0 || ukr.x >= cols || ukr.y >= rows || vcurr_cp.z < 0)
            return false;

        float3 vprev_g;
        vprev_g.x = vmap_g_prev.ptr (ukr.y       )[ukr.x];
        vprev_g.y = vmap_g_prev.ptr (ukr.y + rows)[ukr.x];
        vprev_g.z = vmap_g_prev.ptr (ukr.y + 2 * rows)[ukr.x];

        float3 ncurr;
        ncurr.x = nmap_curr.ptr (y)[x];
        ncurr.y = nmap_curr.ptr (y + rows)[x];
        ncurr.z = nmap_curr.ptr (y + 2 * rows)[x];

        float3 ncurr_g = Rcurr * ncurr;

        float3 nprev_g;
        nprev_g.x = nmap_g_prev.ptr (ukr.y)[ukr.x];
        nprev_g.y = nmap_g_prev.ptr (ukr.y + rows)[ukr.x];
        nprev_g.z = nmap_g_prev.ptr (ukr.y + 2 * rows)[ukr.x];

        float dist = norm (vprev_g - vcurr_g);
        float sine = norm (cross (ncurr_g, nprev_g));

        n = nprev_g;
        d = vprev_g;
        s = vcurr_g;

        return (sine < angleThres && dist <= distThres && !isnan (ncurr.x) && !isnan (nprev_g.x));
    }

    __device__ __forceinline__ JtJJtrSE3
    getProducts(int & i) const
    {
        int y = i / cols;
        int x = i - (y * cols);

        float3 n_cp, d_cp, s_cp;

        bool found_coresp = search (x, y, n_cp, d_cp, s_cp);

        float row[7] = {0, 0, 0, 0, 0, 0, 0};

        if(found_coresp)
        {
            s_cp = Rprev_inv * (s_cp - tprev);
            d_cp = Rprev_inv * (d_cp - tprev);
            n_cp = Rprev_inv * (n_cp);

            *(float3*)&row[0] = n_cp;
            *(float3*)&row[3] = cross (s_cp, n_cp);
            row[6] = dot (n_cp, s_cp - d_cp);
        }

        JtJJtrSE3 values = {row[0] * row[0],
                            row[0] * row[1],
                            row[0] * row[2],
                            row[0] * row[3],
                            row[0] * row[4],
                            row[0] * row[5],
                            row[0] * row[6],

                            row[1] * row[1],
                            row[1] * row[2],
                            row[1] * row[3],
                            row[1] * row[4],
                            row[1] * row[5],
                            row[1] * row[6],

                            row[2] * row[2],
                            row[2] * row[3],
                            row[2] * row[4],
                            row[2] * row[5],
                            row[2] * row[6],

                            row[3] * row[3],
                            row[3] * row[4],
                            row[3] * row[5],
                            row[3] * row[6],

                            row[4] * row[4],
                            row[4] * row[5],
                            row[4] * row[6],

                            row[5] * row[5],
                            row[5] * row[6],

                            row[6] * row[6],
                            float(found_coresp)};

        return values;
    }

    __device__ __forceinline__ void
    operator () () const
    {
        JtJJtrSE3 sum = {0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0};

        for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x)
        {
            JtJJtrSE3 val = getProducts(i);

            sum.add(val);
        }

        sum = blockReduceSum(sum);

        if(threadIdx.x == 0)
        {
            out[blockIdx.x] = sum;
        }
    }
};

__global__ void icpKernel(const ICPReduction icp)
{
    icp();
}

void icpStep(const mat33& Rcurr,
             const float3& tcurr,
             const DeviceArray2D<float>& vmap_curr,
             const DeviceArray2D<float>& nmap_curr,
             const mat33& Rprev_inv,
             const float3& tprev,
             const CameraModel& intr,
             const DeviceArray2D<float>& vmap_g_prev,
             const DeviceArray2D<float>& nmap_g_prev,
             float distThres,
             float angleThres,
             DeviceArray<JtJJtrSE3> & sum,
             DeviceArray<JtJJtrSE3> & out,
             float * matrixA_host,
             float * vectorB_host,
             float * residual_host)
{
    int cols = vmap_curr.cols ();
    int rows = vmap_curr.rows () / 3;

    ICPReduction icp;

    icp.Rcurr = Rcurr;
    icp.tcurr = tcurr;

    icp.vmap_curr = vmap_curr;
    icp.nmap_curr = nmap_curr;

    icp.Rprev_inv = Rprev_inv;
    icp.tprev = tprev;

    icp.intr = intr;

    icp.vmap_g_prev = vmap_g_prev;
    icp.nmap_g_prev = nmap_g_prev;

    icp.distThres = distThres;
    icp.angleThres = angleThres;

    icp.cols = cols;
    icp.rows = rows;

    icp.N = cols * rows;
    icp.out = sum;

    constexpr int reduceThreads = 256;
    constexpr int reduceBlocks = 80;

    icpKernel<<<reduceBlocks, reduceThreads>>>(icp);

    reduceSum<<<1, MAX_THREADS>>>(sum, out, reduceBlocks);

    cudaSafeCall(cudaGetLastError());
    cudaSafeCall(cudaDeviceSynchronize());

    float host_data[32];
    out.download((JtJJtrSE3 *)&host_data[0]);

    int shift = 0;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 7; ++j)
        {
            float value = host_data[shift++];
            if (j == 6)
                vectorB_host[i] = value;
            else
                matrixA_host[j * 6 + i] = matrixA_host[i * 6 + j] = value;
        }
    }

    residual_host[0] = host_data[27];
    residual_host[1] = host_data[28];
}

#define FLT_EPSILON ((float)1.19209290E-07F)

struct RGBReduction
{
    PtrStepSz<DataTerm> corresImg;

    float sigma;
    PtrStepSz<float3> cloud;
    float fx;
    float fy;
    PtrStepSz<int16_t> dIdx;
    PtrStepSz<int16_t> dIdy;
    float sobelScale;

    int cols;
    int rows;
    int N;

    JtJJtrSE3 * out;

    __device__ __forceinline__ JtJJtrSE3
    getProducts(int & i) const
    {
        const DataTerm & corresp = corresImg.data[i];

        bool found_coresp = corresp.valid;

        float row[7];

        if(found_coresp)
        {
            float w = sigma + std::abs(corresp.diff);

            w = w > FLT_EPSILON ? 1.0f / w : 1.0f;

            //Signals RGB only tracking, so we should only
            if(sigma == -1)
            {
                w = 1;
            }

            row[6] = -w * corresp.diff;

            float3 cloudPoint = {cloud.ptr(corresp.zero.y)[corresp.zero.x].x,
                                 cloud.ptr(corresp.zero.y)[corresp.zero.x].y,
                                 cloud.ptr(corresp.zero.y)[corresp.zero.x].z};

            float invz = 1.0 / cloudPoint.z;
            float dI_dx_val = w * sobelScale * dIdx.ptr(corresp.one.y)[corresp.one.x];
            float dI_dy_val = w * sobelScale * dIdy.ptr(corresp.one.y)[corresp.one.x];
            float v0 = dI_dx_val * fx * invz;
            float v1 = dI_dy_val * fy * invz;
            float v2 = -(v0 * cloudPoint.x + v1 * cloudPoint.y) * invz;

            row[0] = v0;
            row[1] = v1;
            row[2] = v2;
            row[3] = -cloudPoint.z * v1 + cloudPoint.y * v2;
            row[4] =  cloudPoint.z * v0 - cloudPoint.x * v2;
            row[5] = -cloudPoint.y * v0 + cloudPoint.x * v1;
        }
        else
        {
            row[0] = row[1] = row[2] = row[3] = row[4] = row[5] = row[6] = 0.f;
        }

        JtJJtrSE3 values = {row[0] * row[0],
                            row[0] * row[1],
                            row[0] * row[2],
                            row[0] * row[3],
                            row[0] * row[4],
                            row[0] * row[5],
                            row[0] * row[6],

                            row[1] * row[1],
                            row[1] * row[2],
                            row[1] * row[3],
                            row[1] * row[4],
                            row[1] * row[5],
                            row[1] * row[6],

                            row[2] * row[2],
                            row[2] * row[3],
                            row[2] * row[4],
                            row[2] * row[5],
                            row[2] * row[6],

                            row[3] * row[3],
                            row[3] * row[4],
                            row[3] * row[5],
                            row[3] * row[6],

                            row[4] * row[4],
                            row[4] * row[5],
                            row[4] * row[6],

                            row[5] * row[5],
                            row[5] * row[6],

                            row[6] * row[6],
                            float(found_coresp)};

        return values;
    }

    __device__ __forceinline__ void
    operator () () const
    {
        JtJJtrSE3 sum = {0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0};

        for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x)
        {
            JtJJtrSE3 val = getProducts(i);

            sum.add(val);
        }

        sum = blockReduceSum(sum);

        if(threadIdx.x == 0)
        {
            out[blockIdx.x] = sum;
        }
    }
};

__global__ void rgbKernel (const RGBReduction rgb)
{
    rgb();
}

void rgbStep(const DeviceArray2D<DataTerm> & corresImg,
             const float & sigma,
             const DeviceArray2D<float3> & cloud,
             const float & fx,
             const float & fy,
             const DeviceArray2D<int16_t> & dIdx,
             const DeviceArray2D<int16_t> & dIdy,
             const float & sobelScale,
             DeviceArray<JtJJtrSE3> & sum,
             DeviceArray<JtJJtrSE3> & out,
             float * matrixA_host,
             float * vectorB_host)
{
    RGBReduction rgb;

    rgb.corresImg = corresImg;
    rgb.cols = corresImg.cols();
    rgb.rows = corresImg.rows();
    rgb.sigma = sigma;
    rgb.cloud = cloud;
    rgb.fx = fx;
    rgb.fy = fy;
    rgb.dIdx = dIdx;
    rgb.dIdy = dIdy;
    rgb.sobelScale = sobelScale;
    rgb.N = rgb.cols * rgb.rows;
    rgb.out = sum;

    constexpr int reduceThreads = 256;
    constexpr int reduceBlocks = 80;

    rgbKernel<<<reduceBlocks, reduceThreads>>>(rgb);

    reduceSum<<<1, MAX_THREADS>>>(sum, out, reduceBlocks);

    cudaSafeCall(cudaGetLastError());
    cudaSafeCall(cudaDeviceSynchronize());

    float host_data[32];
    out.download((JtJJtrSE3 *)&host_data[0]);

    int shift = 0;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 7; ++j)
        {
            float value = host_data[shift++];
            if (j == 6)
                vectorB_host[i] = value;
            else
                matrixA_host[j * 6 + i] = matrixA_host[i * 6 + j] = value;
        }
    }
}

__inline__  __device__ int2 warpReduceSum(int2 val)
{
    for(int offset = warpSize / 2; offset > 0; offset /= 2)
    {
        val.x += __shfl_down_sync(0xFFFFFFFF, val.x, offset);
        val.y += __shfl_down_sync(0xFFFFFFFF, val.y, offset);
    }

    return val;
}

__inline__  __device__ int2 blockReduceSum(int2 val)
{
    static __shared__ int2 shared[32];

    int lane = threadIdx.x % warpSize;

    int wid = threadIdx.x / warpSize;

    val = warpReduceSum(val);

    //write reduced value to shared memory
    if(lane == 0)
    {
        shared[wid] = val;
    }
    __syncthreads();

    const int2 zero = {0, 0};

    //ensure we only grab a value from shared memory if that warp existed
    val = (threadIdx.x < blockDim.x / warpSize) ? shared[lane] : zero;

    if(wid == 0)
    {
        val = warpReduceSum(val);
    }

    return val;
}

__global__ void reduceSum(int2 * in, int2 * out, int N)
{
    int2 sum = {0, 0};

    for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x)
    {
        sum.x += in[i].x;
        sum.y += in[i].y;
    }

    sum = blockReduceSum(sum);

    if(threadIdx.x == 0)
    {
        out[blockIdx.x] = sum;
    }
}

struct RGBResidual
{
    float minScale;

    PtrStepSz<int16_t> dIdx;
    PtrStepSz<int16_t> dIdy;

    PtrStepSz<float> lastDepth;
    PtrStepSz<float> nextDepth;

    PtrStepSz<uint8_t> lastImage;
    PtrStepSz<uint8_t> nextImage;

    mutable PtrStepSz<DataTerm> corresImg;

    float maxDepthDelta;

    float3 kt;
    mat33 krkinv;

    int cols;
    int rows;
    int N;

    int pitch;
    int imgPitch;

    int2 * out;

    __device__ __forceinline__ int2
    getProducts(int k) const
    {
        int i = k / cols;
        int j0 = k - (i * cols);

        int2 value = {0, 0};

        DataTerm corres;

        corres.valid = false;

        if(i >= 0 && i < rows && j0 >= 0 && j0 < cols)
        {
            if(j0 < cols - 5 && i < rows - 1)
            {
                bool valid = true;

                for(int u = max(i - 2, 0); u < min(i + 2, rows); u++)
                {
                    for(int v = max(j0 - 2, 0); v < min(j0 + 2, cols); v++)
                    {
                        valid = valid && (nextImage.ptr(u)[v] > 0);
                    }
                }

                if(valid)
                {
                    int16_t * ptr_input_x = (int16_t*) ((uint8_t*) dIdx.data + i * pitch);
                    int16_t * ptr_input_y = (int16_t*) ((uint8_t*) dIdy.data + i * pitch);

                    int16_t valx = ptr_input_x[j0];
                    int16_t valy = ptr_input_y[j0];
                    float mTwo = (valx * valx) + (valy * valy);

                    if(mTwo >= minScale)
                    {
                        int y = i;
                        int x = j0;

                        float d1 = nextDepth.ptr(y)[x];

                        if(!isnan(d1))
                        {
                            float transformed_d1 = (float)(d1 * (krkinv.data[2].x * x + krkinv.data[2].y * y + krkinv.data[2].z) + kt.z);
                            int u0 = __float2int_rn((d1 * (krkinv.data[0].x * x + krkinv.data[0].y * y + krkinv.data[0].z) + kt.x) / transformed_d1);
                            int v0 = __float2int_rn((d1 * (krkinv.data[1].x * x + krkinv.data[1].y * y + krkinv.data[1].z) + kt.y) / transformed_d1);

                            if(u0 >= 0 && v0 >= 0 && u0 < lastDepth.cols && v0 < lastDepth.rows)
                            {
                                float d0 = lastDepth.ptr(v0)[u0];

                                if(d0 > 0 && std::abs(transformed_d1 - d0) <= maxDepthDelta && lastImage.ptr(v0)[u0] != 0)
                                {
                                    corres.zero.x = u0;
                                    corres.zero.y = v0;
                                    corres.one.x = x;
                                    corres.one.y = y;
                                    corres.diff = static_cast<float>(nextImage.ptr(y)[x]) - static_cast<float>(lastImage.ptr(v0)[u0]);
                                    corres.valid = true;
                                    value.x = 1;
                                    value.y = corres.diff * corres.diff;
                                }
                            }
                        }
                    }
                }
            }
        }

        corresImg.data[k] = corres;

        return value;
    }

    __device__ __forceinline__ void
    operator () () const
    {
        int2 sum = {0, 0};

        for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x)
        {
            int2 val = getProducts(i);
            sum.x += val.x;
            sum.y += val.y;
        }

        sum = blockReduceSum(sum);

        if(threadIdx.x == 0)
        {
            out[blockIdx.x] = sum;
        }
    }
};

__global__ void residualKernel (const RGBResidual rgb)
{
    rgb();
}

void computeRgbResidual(const float & minScale,
                        const DeviceArray2D<int16_t> & dIdx,
                        const DeviceArray2D<int16_t> & dIdy,
                        const DeviceArray2D<float> & lastDepth,
                        const DeviceArray2D<float> & nextDepth,
                        const DeviceArray2D<uint8_t> & lastImage,
                        const DeviceArray2D<uint8_t> & nextImage,
                        DeviceArray2D<DataTerm> & corresImg,
                        DeviceArray<int2> & sumResidual,
                        const float maxDepthDelta,
                        const float3 & kt,
                        const mat33 & krkinv,
                        int & sigmaSum,
                        int & count)
{
    int cols = nextImage.cols ();
    int rows = nextImage.rows ();

    RGBResidual rgb;

    rgb.minScale = minScale;

    rgb.dIdx = dIdx;
    rgb.dIdy = dIdy;

    rgb.lastDepth = lastDepth;
    rgb.nextDepth = nextDepth;

    rgb.lastImage = lastImage;
    rgb.nextImage = nextImage;

    rgb.corresImg = corresImg;

    rgb.maxDepthDelta = maxDepthDelta;

    rgb.kt = kt;
    rgb.krkinv = krkinv;

    rgb.cols = cols;
    rgb.rows = rows;
    rgb.pitch = dIdx.step();
    rgb.imgPitch = nextImage.step();

    rgb.N = cols * rows;
    rgb.out = sumResidual;

    constexpr int reduceThreads = 256;
    constexpr int reduceBlocks = 80;

    residualKernel<<<reduceBlocks, reduceThreads>>>(rgb);

    int2 out_host = {0, 0};
    int2 * out;

    cudaMalloc(&out, sizeof(int2));
    cudaMemcpy(out, &out_host, sizeof(int2), cudaMemcpyHostToDevice);

    reduceSum<<<1, MAX_THREADS>>>(sumResidual, out, reduceBlocks);

    cudaSafeCall(cudaGetLastError());
    cudaSafeCall(cudaDeviceSynchronize());

    cudaMemcpy(&out_host, out, sizeof(int2), cudaMemcpyDeviceToHost);
    cudaFree(out);

    count = out_host.x;
    sigmaSum = out_host.y;
}

struct SO3Reduction
{
    PtrStepSz<uint8_t> lastImage;
    PtrStepSz<uint8_t> nextImage;

    mat33 imageBasis;
    mat33 kinv;
    mat33 krlr;
    bool gradCheck;

    int cols;
    int rows;
    int N;

    JtJJtrSO3 * out;

    __device__ __forceinline__ float2
    getGradient(const PtrStepSz<uint8_t> img, int x, int y) const
    {
        float2 gradient;

        float actu = static_cast<float>(img.ptr(y)[x]);

        float back = static_cast<float>(img.ptr(y)[x - 1]);
        float fore = static_cast<float>(img.ptr(y)[x + 1]);
        gradient.x = ((back + actu) / 2.0f) - ((fore + actu) / 2.0f);

        back = static_cast<float>(img.ptr(y - 1)[x]);
        fore = static_cast<float>(img.ptr(y + 1)[x]);
        gradient.y = ((back + actu) / 2.0f) - ((fore + actu) / 2.0f);

        return gradient;
    }

    __device__ __forceinline__ JtJJtrSO3
    getProducts(int k) const
    {
        int y = k / cols;
        int x = k - (y * cols);

        bool found_coresp = false;

        float3 unwarpedReferencePoint = {float(x), float(y), 1.0f};

        float3 warpedReferencePoint = imageBasis * unwarpedReferencePoint;

        int2 warpedReferencePixel = {__float2int_rn(warpedReferencePoint.x / warpedReferencePoint.z),
                                     __float2int_rn(warpedReferencePoint.y / warpedReferencePoint.z)};

        if(warpedReferencePixel.x >= 1 &&
           warpedReferencePixel.x < cols - 1 &&
           warpedReferencePixel.y >= 1 &&
           warpedReferencePixel.y < rows - 1 &&
           x >= 1 &&
           x < cols - 1 &&
           y >= 1 &&
           y < rows - 1)
        {
            found_coresp = true;
        }

        float row[4];
        row[0] = row[1] = row[2] = row[3] = 0.f;

        if(found_coresp)
        {
            float2 gradNext = getGradient(nextImage, warpedReferencePixel.x, warpedReferencePixel.y);
            float2 gradLast = getGradient(lastImage, x, y);

            float gx = (gradNext.x + gradLast.x) / 2.0f;
            float gy = (gradNext.y + gradLast.y) / 2.0f;

            float3 point = kinv * unwarpedReferencePoint;

            float z2 = point.z * point.z;

            float a = krlr.data[0].x;
            float b = krlr.data[0].y;
            float c = krlr.data[0].z;

            float d = krlr.data[1].x;
            float e = krlr.data[1].y;
            float f = krlr.data[1].z;

            float g = krlr.data[2].x;
            float h = krlr.data[2].y;
            float i = krlr.data[2].z;

            //Aren't jacobians great fun
            float3 leftProduct = {((point.z * (d * gy + a * gx)) - (gy * g * y) - (gx * g * x)) / z2,
                                  ((point.z * (e * gy + b * gx)) - (gy * h * y) - (gx * h * x)) / z2,
                                  ((point.z * (f * gy + c * gx)) - (gy * i * y) - (gx * i * x)) / z2};

            float3 jacRow = cross(leftProduct, point);

            row[0] = jacRow.x;
            row[1] = jacRow.y;
            row[2] = jacRow.z;
            row[3] = -(static_cast<float>(nextImage.ptr(warpedReferencePixel.y)[warpedReferencePixel.x]) - static_cast<float>(lastImage.ptr(y)[x]));
        }

        JtJJtrSO3 values = {row[0] * row[0],
                            row[0] * row[1],
                            row[0] * row[2],
                            row[0] * row[3],

                            row[1] * row[1],
                            row[1] * row[2],
                            row[1] * row[3],

                            row[2] * row[2],
                            row[2] * row[3],

                            row[3] * row[3],
                            float(found_coresp)};

        return values;
    }

    __device__ __forceinline__ void
    operator () () const
    {
        JtJJtrSO3 sum = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x)
        {
            JtJJtrSO3 val = getProducts(i);

            sum.add(val);
        }

        sum = blockReduceSum(sum);

        if(threadIdx.x == 0)
        {
            out[blockIdx.x] = sum;
        }
    }
};

__global__ void so3Kernel (const SO3Reduction so3)
{
    so3();
}

void so3Step(const DeviceArray2D<uint8_t> & lastImage,
             const DeviceArray2D<uint8_t> & nextImage,
             const mat33 & imageBasis,
             const mat33 & kinv,
             const mat33 & krlr,
             DeviceArray<JtJJtrSO3> & sum,
             DeviceArray<JtJJtrSO3> & out,
             float * matrixA_host,
             float * vectorB_host,
             float * residual_host)
{
    int cols = nextImage.cols();
    int rows = nextImage.rows();

    SO3Reduction so3;

    so3.lastImage = lastImage;

    so3.nextImage = nextImage;

    so3.imageBasis = imageBasis;
    so3.kinv = kinv;
    so3.krlr = krlr;

    so3.cols = cols;
    so3.rows = rows;

    so3.N = cols * rows;

    so3.out = sum;

    constexpr int reduceThreads = 256;
    constexpr int reduceBlocks = 80;

    so3Kernel<<<reduceBlocks, reduceThreads>>>(so3);

    reduceSum<<<1, MAX_THREADS>>>(sum, out, reduceBlocks);

    cudaSafeCall(cudaGetLastError());
    cudaSafeCall(cudaDeviceSynchronize());

    float host_data[11];
    out.download((JtJJtrSO3 *)&host_data[0]);

    int shift = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = i; j < 4; ++j)
        {
            float value = host_data[shift++];
            if (j == 3)
                vectorB_host[i] = value;
            else
                matrixA_host[j * 3 + i] = matrixA_host[i * 3 + j] = value;
        }
    }

    residual_host[0] = host_data[9];
    residual_host[1] = host_data[10];
}
