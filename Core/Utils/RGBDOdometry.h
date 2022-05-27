/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at
 * <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef RGBDODOMETRY_H_
#define RGBDODOMETRY_H_

#include "../Cuda/cudafuncs.cuh"
#include "../GPUTexture.h"
#include "OdometryProvider.h"
#include "Stopwatch.h"

#include <vector_types.h>
#include <vector>

class RGBDOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RGBDOdometry(
      int width,
      int height,
      float cx,
      float cy,
      float fx,
      float fy,
      float distThresh = 0.10f,
      float angleThresh = sin(20.f * 3.14159254f / 180.f));

  virtual ~RGBDOdometry();

  void initICP(GPUTexture* filteredDepth, const float depthCutoff);

  void initICP(GPUTexture* predictedVertices, GPUTexture* predictedNormals);

  void initICPModel(
      GPUTexture* predictedVertices,
      GPUTexture* predictedNormals,
      const Sophus::SE3d& T_wc);

  void initRGB(GPUTexture* rgb);

  void initRGBModel(GPUTexture* rgb);

  void initFirstRGB(GPUTexture* rgb);

  void getIncrementalTransformation(
      Sophus::SE3d& T_wc,
      const bool& rgbOnly,
      const float& icpWeight,
      const bool& pyramid,
      const bool& fastOdom,
      const bool& so3);

  Eigen::MatrixXd getCovariance();

  float lastICPError;
  float lastICPCount;
  float lastRGBError;
  float lastRGBCount;
  float lastSO3Error;
  float lastSO3Count;

  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> lastA;
  Eigen::Matrix<double, 6, 1> lastb;

 private:
  void populateRGBDData(
      GPUTexture* rgb,
      DeviceArray2D<float>* destDepths,
      DeviceArray2D<uint8_t>* destImages);

  std::vector<DeviceArray2D<uint16_t>> depth_tmp;

  DeviceArray<float> vmaps_tmp;
  DeviceArray<float> nmaps_tmp;

  std::vector<DeviceArray2D<float>> vmaps_g_prev_;
  std::vector<DeviceArray2D<float>> nmaps_g_prev_;

  std::vector<DeviceArray2D<float>> vmaps_curr_;
  std::vector<DeviceArray2D<float>> nmaps_curr_;

  CameraModel intr;

  DeviceArray<JtJJtrSE3> sumDataSE3;
  DeviceArray<JtJJtrSE3> outDataSE3;
  DeviceArray<int2> sumResidualRGB;

  DeviceArray<JtJJtrSO3> sumDataSO3;
  DeviceArray<JtJJtrSO3> outDataSO3;

  const int sobelSize;
  const float sobelScale;
  const float maxDepthDeltaRGB;
  const float maxDepthRGB;

  std::vector<int2> pyrDims;

  static constexpr int NUM_PYRS = 3;

  DeviceArray2D<float> lastDepth[NUM_PYRS];
  DeviceArray2D<uint8_t> lastImage[NUM_PYRS];

  DeviceArray2D<float> nextDepth[NUM_PYRS];
  DeviceArray2D<uint8_t> nextImage[NUM_PYRS];
  DeviceArray2D<int16_t> nextdIdx[NUM_PYRS];
  DeviceArray2D<int16_t> nextdIdy[NUM_PYRS];

  DeviceArray2D<uint8_t> lastNextImage[NUM_PYRS];

  DeviceArray2D<DataTerm> corresImg[NUM_PYRS];

  DeviceArray2D<float3> pointClouds[NUM_PYRS];

  std::vector<int> iterations;
  std::vector<float> minimumGradientMagnitudes;

  float distThres_;
  float angleThres_;

  Eigen::Matrix<double, 6, 6> lastCov;

  const int width;
  const int height;
  const float cx, cy, fx, fy;
};

#endif /* RGBDODOMETRY_H_ */
