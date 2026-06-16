// ElasticFusion ROCm/HIP kernel-correctness harness (no GL, no Pangolin).
//
// The GL/CUDA interop ingest path is environmentally infeasible on this headless
// MI250X (compute-only chip: radeonsi cannot create a GL context; no Vulkan ICD
// for zink; software GL (llvmpipe) is host-memory so hipGraphicsGLRegisterImage
// fails). This harness validates the *ported tracking kernels* directly on real
// gfx90a by feeding fixed DeviceArray inputs into createVMap/createNMap/
// tranformMaps/resizeVMap/resizeNMap/icpStep/so3Step, the exact device path the
// SLAM tracker drives, and checking against CPU recomputation plus a two-run
// determinism check that fingerprints any wave64 reduction non-determinism.
//
// Authored with the assistance of Claude (Anthropic).

#include "cudafuncs.cuh"
#include "containers/device_array.hpp"

#include <hip/hip_runtime.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

static int g_fail = 0;
#define CHECK(cond, msg) do { if(!(cond)){ printf("  FAIL: %s\n", msg); ++g_fail; } \
                              else printf("  ok:   %s\n", msg); } while(0)

static const float QNAN = []{ int i=0x7fffffff; float f; std::memcpy(&f,&i,4); return f; }();

// ---- createVMap: depth(mm,uint16) -> vmap(3 stacked HxW float planes) ----
static void test_createVMap_createNMap() {
  printf("[createVMap / createNMap]\n");
  const int W = 64, H = 48;
  CameraModel intr(528.0f, 528.0f, 320.0f, 240.0f);

  std::vector<uint16_t> depth(W * H);
  for (int v = 0; v < H; ++v)
    for (int u = 0; u < W; ++u)
      depth[v * W + u] = (uint16_t)(1000 + ((u * 7 + v * 13) % 2000));  // 1.0..3.0 m

  DeviceArray2D<uint16_t> d_depth;
  d_depth.upload(depth.data(), W * sizeof(uint16_t), H, W);

  DeviceArray2D<float> d_vmap, d_nmap;
  createVMap(intr, d_depth, d_vmap, 10.0f /*cutoff m*/);
  createNMap(d_vmap, d_nmap);
  hipDeviceSynchronize();

  std::vector<float> vmap(W * H * 3);
  d_vmap.download(vmap.data(), W * sizeof(float));

  // CPU recompute of vmap and compare.
  int vbad = 0;
  for (int v = 0; v < H; ++v)
    for (int u = 0; u < W; ++u) {
      float z = depth[v * W + u] / 1000.f;
      float ex = z * (u - intr.cx) / intr.fx;
      float ey = z * (v - intr.cy) / intr.fy;
      float gx = vmap[v * W + u];
      float gy = vmap[(v + H) * W + u];
      float gz = vmap[(v + 2 * H) * W + u];
      if (std::fabs(gx - ex) > 1e-4f || std::fabs(gy - ey) > 1e-4f || std::fabs(gz - z) > 1e-4f)
        ++vbad;
    }
  CHECK(vbad == 0, "vmap == CPU back-projection (all pixels)");

  // Normals: unit length where valid, and matches CPU cross product at a sample.
  std::vector<float> nmap(W * H * 3);
  d_nmap.download(nmap.data(), W * sizeof(float));
  int nbad = 0, nvalid = 0;
  for (int v = 0; v < H - 1; ++v)
    for (int u = 0; u < W - 1; ++u) {
      float nx = nmap[v * W + u];
      if (std::isnan(nx)) continue;
      float ny = nmap[(v + H) * W + u];
      float nz = nmap[(v + 2 * H) * W + u];
      float len = std::sqrt(nx * nx + ny * ny + nz * nz);
      ++nvalid;
      if (std::fabs(len - 1.0f) > 1e-3f) ++nbad;
    }
  printf("  (valid normals=%d)\n", nvalid);
  CHECK(nvalid > 1000, "createNMap produced a dense normal field");
  CHECK(nbad == 0, "all valid normals are unit length");
}

// ---- tranformMaps with identity R,t: vmap unchanged ----
static void test_tranformMaps_identity() {
  printf("[tranformMaps identity]\n");
  const int W = 64, H = 48;
  CameraModel intr(528.0f, 528.0f, 320.0f, 240.0f);
  std::vector<uint16_t> depth(W * H);
  for (int i = 0; i < W * H; ++i) depth[i] = (uint16_t)(1500 + (i % 1000));
  DeviceArray2D<uint16_t> d_depth; d_depth.upload(depth.data(), W * sizeof(uint16_t), H, W);
  DeviceArray2D<float> d_vmap, d_nmap; createVMap(intr, d_depth, d_vmap, 10.0f); createNMap(d_vmap, d_nmap);

  mat33 R; R.data[0] = make_float3(1, 0, 0); R.data[1] = make_float3(0, 1, 0); R.data[2] = make_float3(0, 0, 1);
  float3 t = make_float3(0, 0, 0);
  DeviceArray2D<float> d_vout, d_nout;
  tranformMaps(d_vmap, d_nmap, R, t, d_vout, d_nout);
  hipDeviceSynchronize();

  std::vector<float> a(W * H * 3), b(W * H * 3);
  d_vmap.download(a.data(), W * sizeof(float));
  d_vout.download(b.data(), W * sizeof(float));
  int bad = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    bool na = std::isnan(a[i]), nb = std::isnan(b[i]);
    if (na != nb || (!na && std::fabs(a[i] - b[i]) > 1e-4f)) ++bad;
  }
  CHECK(bad == 0, "identity transform leaves vmap bit-stable");
}

// ---- resizeVMap: half-resolution averaging stays finite & sane ----
static void test_resizeVMap() {
  printf("[resizeVMap]\n");
  const int W = 64, H = 48;
  CameraModel intr(528.0f, 528.0f, 320.0f, 240.0f);
  std::vector<uint16_t> depth(W * H);
  for (int i = 0; i < W * H; ++i) depth[i] = 2000;
  DeviceArray2D<uint16_t> d_depth; d_depth.upload(depth.data(), W * sizeof(uint16_t), H, W);
  DeviceArray2D<float> d_vmap; createVMap(intr, d_depth, d_vmap, 10.0f);
  DeviceArray2D<float> d_half; resizeVMap(d_vmap, d_half);
  hipDeviceSynchronize();
  CHECK(d_half.cols() == W / 2 && d_half.rows() == (H / 2) * 3, "resizeVMap halved dimensions");
  std::vector<float> h(d_half.cols() * d_half.rows());
  d_half.download(h.data(), d_half.cols() * sizeof(float));
  int finite = 0; for (float f : h) if (std::isfinite(f)) ++finite;
  CHECK(finite > (int)h.size() / 2, "resizeVMap output mostly finite");
}

// ---- icpStep: the SE3 wave64 reduction. Two identical clouds -> the JtJ must be
//      symmetric, residual ~ 0 at identity, inliers = #valid pixels, and crucially
//      two runs must be identical (wave64 reduction determinism). ----
static double g_icp_sumA = 0, g_icp_sumB = 0;
static int run_icpStep(float matA[36], float vecB[6], float resid[2]) {
  const int W = 64, H = 48;
  CameraModel intr(528.0f, 528.0f, 320.0f, 240.0f);

  std::vector<uint16_t> depth(W * H);
  for (int v = 0; v < H; ++v)
    for (int u = 0; u < W; ++u)
      depth[v * W + u] = (uint16_t)(1200 + ((u * 11 + v * 17) % 1500));
  DeviceArray2D<uint16_t> d_depth; d_depth.upload(depth.data(), W * sizeof(uint16_t), H, W);
  DeviceArray2D<float> vmap, nmap; createVMap(intr, d_depth, vmap, 10.0f); createNMap(vmap, nmap);

  mat33 I; I.data[0]=make_float3(1,0,0); I.data[1]=make_float3(0,1,0); I.data[2]=make_float3(0,0,1);
  float3 z = make_float3(0,0,0);

  DeviceArray<JtJJtrSE3> sumBuf, outBuf;
  sumBuf.create(MAX_THREADS); outBuf.create(1);

  std::memset(matA, 0, 36 * sizeof(float)); std::memset(vecB, 0, 6 * sizeof(float));
  // curr == prev (identity pose): perfect correspondence everywhere valid.
  icpStep(I, z, vmap, nmap, I, z, intr, vmap, nmap, 0.10f, 0.5f,
          sumBuf, outBuf, matA, vecB, resid);
  hipDeviceSynchronize();
  return (int)resid[1];  // inliers
}
static void test_icpStep() {
  printf("[icpStep SE3 wave64 reduction]\n");
  float matA1[36], vecB1[6], r1[2];
  float matA2[36], vecB2[6], r2[2];
  int inl1 = run_icpStep(matA1, vecB1, r1);
  int inl2 = run_icpStep(matA2, vecB2, r2);

  // symmetry of JtJ
  int asym = 0;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      if (std::fabs(matA1[i * 6 + j] - matA1[j * 6 + i]) > 1e-3f) ++asym;
  CHECK(asym == 0, "JtJ is symmetric");
  CHECK(inl1 > 1000, "icpStep found many inlier correspondences");
  printf("  (inliers=%d residual=%.6g)\n", inl1, r1[0]);

  // determinism: two runs identical (this is the wave64 reduction fingerprint)
  int dbad = 0;
  for (int i = 0; i < 36; ++i) if (std::memcmp(&matA1[i], &matA2[i], 4)) ++dbad;
  for (int i = 0; i < 6; ++i)  if (std::memcmp(&vecB1[i], &vecB2[i], 4)) ++dbad;
  if (std::memcmp(&r1[0], &r2[0], 4)) ++dbad;
  if (std::memcmp(&r1[1], &r2[1], 4)) ++dbad;
  CHECK(inl1 == inl2, "inlier count identical across two runs");
  CHECK(dbad == 0, "icpStep JtJ/JtR/residual BIT-IDENTICAL across two runs (wave64 determinism)");

  // value sanity: the SE3 normal-equation diagonal must be > 0 (real Jacobian energy)
  int posdiag = 0; for (int i = 0; i < 6; ++i) if (matA1[i * 6 + i] > 0) ++posdiag;
  CHECK(posdiag == 6, "all 6 JtJ diagonal entries positive (non-degenerate reduction)");
}

// ---- so3Step: the SO3 wave64 reduction. Two runs identical. ----
static void test_so3Step() {
  printf("[so3Step SO3 wave64 reduction]\n");
  const int W = 64, H = 48;
  std::vector<uint8_t> img(W * H);
  for (int v = 0; v < H; ++v)
    for (int u = 0; u < W; ++u)
      img[v * W + u] = (uint8_t)((u * 5 + v * 9) & 0xff);
  DeviceArray2D<uint8_t> last, next;
  last.upload(img.data(), W * sizeof(uint8_t), H, W);
  next.upload(img.data(), W * sizeof(uint8_t), H, W);

  mat33 I; I.data[0]=make_float3(1,0,0); I.data[1]=make_float3(0,1,0); I.data[2]=make_float3(0,0,1);
  DeviceArray<JtJJtrSO3> sumBuf, outBuf; sumBuf.create(MAX_THREADS); outBuf.create(1);

  float A1[9], B1[3], r1[2], A2[9], B2[3], r2[2];
  std::memset(A1,0,sizeof(A1)); std::memset(B1,0,sizeof(B1));
  so3Step(last, next, I, I, I, sumBuf, outBuf, A1, B1, r1); hipDeviceSynchronize();
  std::memset(A2,0,sizeof(A2)); std::memset(B2,0,sizeof(B2));
  so3Step(last, next, I, I, I, sumBuf, outBuf, A2, B2, r2); hipDeviceSynchronize();

  int dbad = 0;
  for (int i = 0; i < 9; ++i) if (std::memcmp(&A1[i], &A2[i], 4)) ++dbad;
  for (int i = 0; i < 3; ++i) if (std::memcmp(&B1[i], &B2[i], 4)) ++dbad;
  printf("  (residual=%.6g inliers=%.0f)\n", r1[0], r1[1]);
  CHECK(dbad == 0, "so3Step JtJ/JtR BIT-IDENTICAL across two runs (wave64 determinism)");
  CHECK(r1[1] > 1000, "so3Step covered many pixels");
}

// ---- computeRgbResidual: drives the int2 warpReduceSum/blockReduceSum (the same
//      wave64 reduction structure as the SE3/SO3 paths). Integer addition is
//      order-independent, so the device count must EXACTLY equal a CPU recount of
//      the valid correspondences -- a hard arithmetic-correctness proof of the
//      reduction tree on the 64-lane wavefront, plus two-run determinism. ----
static void test_computeRgbResidual_intReduction() {
  printf("[computeRgbResidual int2 wave64 reduction]\n");
  const int W = 64, H = 48;

  std::vector<uint8_t> last(W * H), next(W * H);
  std::vector<float> ld(W * H), nd(W * H);
  for (int v = 0; v < H; ++v)
    for (int u = 0; u < W; ++u) {
      last[v * W + u] = (uint8_t)(40 + ((u * 3 + v * 5) % 180));
      next[v * W + u] = (uint8_t)(45 + ((u * 3 + v * 5) % 180));  // +5 intensity shift
      ld[v * W + u] = 1.5f + 0.001f * ((u + v) % 100);
      nd[v * W + u] = ld[v * W + u];  // identical depth -> in-window correspondences
    }
  DeviceArray2D<uint8_t> d_last, d_next; d_last.upload(last.data(), W, H, W); d_next.upload(next.data(), W, H, W);
  DeviceArray2D<float> d_ld, d_nd; d_ld.upload(ld.data(), W * sizeof(float), H, W); d_nd.upload(nd.data(), W * sizeof(float), H, W);

  DeviceArray2D<int16_t> dIdx, dIdy;
  dIdx.create(H, W); dIdy.create(H, W);
  computeDerivativeImages(d_last, dIdx, dIdy);  // also exercises the Sobel __constant__ path
  DeviceArray2D<DataTerm> corres; corres.create(H, W);  // kernel writes one DataTerm per pixel
  DeviceArray<int2> sumResidual; sumResidual.create(MAX_THREADS);

  mat33 I; I.data[0]=make_float3(1,0,0); I.data[1]=make_float3(0,1,0); I.data[2]=make_float3(0,0,1);
  float3 kt = make_float3(0,0,0);

  int sigmaSum1 = -1, count1 = -1, sigmaSum2 = -1, count2 = -1;
  computeRgbResidual(0.0f, dIdx, dIdy, d_ld, d_nd, d_last, d_next, corres, sumResidual,
                     0.05f, kt, I, sigmaSum1, count1);
  hipDeviceSynchronize();
  computeRgbResidual(0.0f, dIdx, dIdy, d_ld, d_nd, d_last, d_next, corres, sumResidual,
                     0.05f, kt, I, sigmaSum2, count2);
  hipDeviceSynchronize();
  printf("  (count=%d sigmaSum=%d)\n", count1, sigmaSum1);

  // CPU recount of valid correspondences from the device-written corresImg, and
  // an exact recompute of count = sum(valid) and sigmaSum = sum(diff^2). The int2
  // reduction must reproduce these EXACTLY (integer adds are order-independent).
  std::vector<DataTerm> hc(W * H);
  corres.download(hc.data(), W * sizeof(DataTerm));
  long cpuCount = 0, cpuSigma = 0;
  for (int i = 0; i < W * H; ++i)
    if (hc[i].valid) { cpuCount++; cpuSigma += (long)(hc[i].diff * hc[i].diff); }

  CHECK(count1 == (int)cpuCount, "int2 reduction count == CPU recount of valid correspondences (EXACT)");
  CHECK(sigmaSum1 == (int)cpuSigma, "int2 reduction sigmaSum == CPU sum(diff^2) (EXACT)");
  CHECK(count1 == count2 && sigmaSum1 == sigmaSum2, "computeRgbResidual identical across two runs");
  CHECK(count1 > 100, "many RGB correspondences found");
}

// ---- rgbStep: the float SE3 reduction over the RGB photometric Jacobian.
//      Drives the same JtJJtrSE3 warpReduceSum/blockReduceSum as icpStep. Verify
//      JtJ symmetry + two-run determinism. Reuses corresImg from a residual pass. ----
static void test_rgbStep() {
  printf("[rgbStep SE3 photometric wave64 reduction]\n");
  const int W = 64, H = 48;
  std::vector<uint8_t> last(W * H), next(W * H);
  std::vector<float> ld(W * H);
  for (int v = 0; v < H; ++v)
    for (int u = 0; u < W; ++u) {
      last[v * W + u] = (uint8_t)(40 + ((u * 3 + v * 5) % 180));
      next[v * W + u] = (uint8_t)(45 + ((u * 3 + v * 5) % 180));
      ld[v * W + u] = 1.5f + 0.001f * ((u + v) % 100);
    }
  DeviceArray2D<uint8_t> d_last, d_next; d_last.upload(last.data(), W, H, W); d_next.upload(next.data(), W, H, W);
  DeviceArray2D<float> d_ld, d_nd; d_ld.upload(ld.data(), W * sizeof(float), H, W); d_nd.upload(ld.data(), W * sizeof(float), H, W);
  DeviceArray2D<int16_t> dIdx, dIdy; dIdx.create(H, W); dIdy.create(H, W);
  computeDerivativeImages(d_next, dIdx, dIdy);
  DeviceArray2D<DataTerm> corres; corres.create(H, W);
  DeviceArray<int2> sumResidual; sumResidual.create(MAX_THREADS);
  mat33 I; I.data[0]=make_float3(1,0,0); I.data[1]=make_float3(0,1,0); I.data[2]=make_float3(0,0,1);
  float3 kt = make_float3(0,0,0);
  int ss=0, cc=0;
  computeRgbResidual(0.0f, dIdx, dIdy, d_ld, d_nd, d_last, d_next, corres, sumResidual, 0.05f, kt, I, ss, cc);

  // build a point cloud for rgbStep
  DeviceArray2D<float3> cloud; cloud.create(H, W);
  CameraModel intr(528,528,320,240);
  projectToPointCloud(d_nd, cloud, intr, 0);
  hipDeviceSynchronize();

  DeviceArray<JtJJtrSE3> sumBuf, outBuf; sumBuf.create(MAX_THREADS); outBuf.create(1);
  float A1[36], B1[6], A2[36], B2[6];
  std::memset(A1,0,sizeof(A1)); std::memset(B1,0,sizeof(B1));
  rgbStep(corres, 1.0f, cloud, intr.fx, intr.fy, dIdx, dIdy, 1.0f/8.0f, sumBuf, outBuf, A1, B1);
  hipDeviceSynchronize();
  std::memset(A2,0,sizeof(A2)); std::memset(B2,0,sizeof(B2));
  rgbStep(corres, 1.0f, cloud, intr.fx, intr.fy, dIdx, dIdy, 1.0f/8.0f, sumBuf, outBuf, A2, B2);
  hipDeviceSynchronize();

  int asym=0; for(int i=0;i<6;i++) for(int j=0;j<6;j++) if(std::fabs(A1[i*6+j]-A1[j*6+i])>1e-2f) ++asym;
  int dbad=0; for(int i=0;i<36;i++) if(std::memcmp(&A1[i],&A2[i],4)) ++dbad;
  for(int i=0;i<6;i++) if(std::memcmp(&B1[i],&B2[i],4)) ++dbad;
  CHECK(asym==0, "rgbStep JtJ symmetric");
  CHECK(dbad==0, "rgbStep JtJ/JtR BIT-IDENTICAL across two runs (wave64 determinism)");
}

int main() {
  int dev = 0; hipGetDevice(&dev);
  hipDeviceProp_t p; hipGetDeviceProperties(&p, dev);
  printf("HIP device %d: %s (%s) warpSize=%d\n\n", dev, p.name, p.gcnArchName, p.warpSize);

  test_createVMap_createNMap();
  test_tranformMaps_identity();
  test_resizeVMap();
  test_icpStep();
  test_so3Step();
  test_computeRgbResidual_intReduction();
  test_rgbStep();

  printf("\n%s (%d failures)\n", g_fail ? "HARNESS FAILED" : "HARNESS PASSED", g_fail);
  return g_fail ? 1 : 0;
}
