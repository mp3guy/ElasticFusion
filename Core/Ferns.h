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

#ifndef FERNS_H_
#define FERNS_H_

#include <Eigen/Core>
#include <Eigen/LU>
#include <limits>
#include <random>
#include <vector>

#include "Shaders/Resize.h"
#include "Utils/Intrinsics.h"
#include "Utils/RGBDOdometry.h"
#include "Utils/Resolution.h"

class Ferns {
 public:
  Ferns(int n, int maxDepth, const float photoThresh);
  virtual ~Ferns();

  bool addFrame(
      GPUTexture* imageTexture,
      GPUTexture* vertexTexture,
      GPUTexture* normalTexture,
      const Sophus::SE3d& T_wc,
      int srcTime,
      const float threshold);

  class SurfaceConstraint {
   public:
    SurfaceConstraint(const Eigen::Vector4d& sourcePoint, const Eigen::Vector4d& targetPoint)
        : sourcePoint(sourcePoint), targetPoint(targetPoint) {}

    Eigen::Vector4d sourcePoint;
    Eigen::Vector4d targetPoint;
  };

  Sophus::SE3d findFrame(
      std::vector<SurfaceConstraint>& constraints,
      const Sophus::SE3d& T_wc,
      GPUTexture* vertexTexture,
      GPUTexture* normalTexture,
      GPUTexture* imageTexture,
      const int time,
      const bool lost);

  class Fern {
   public:
    Fern() {}

    Eigen::Vector2i pos;
    Eigen::Vector4i rgbd;
    std::vector<int> ids[16];
  };

  std::vector<Fern> conservatory;

  class Frame {
   public:
    Frame(
        int n,
        int id,
        const Sophus::SE3d& T_wc,
        const int srcTime,
        const int numPixels,
        uint8_t* rgb = 0,
        Eigen::Vector4f* verts = 0,
        Eigen::Vector4f* norms = 0)
        : goodCodes(0),
          id(id),
          T_wc(T_wc),
          srcTime(srcTime),
          initRgb(rgb),
          initVerts(verts),
          initNorms(norms) {
      codes = new uint8_t[n];

      if (rgb) {
        this->initRgb = new uint8_t[numPixels * 3];
        memcpy(this->initRgb, rgb, numPixels * 3);
      }

      if (verts) {
        this->initVerts = new Eigen::Vector4f[numPixels];
        for (int i = 0; i < numPixels; i++) {
          this->initVerts[i] = verts[i];
        }
      }

      if (norms) {
        this->initNorms = new Eigen::Vector4f[numPixels];
        for (int i = 0; i < numPixels; i++) {
          this->initNorms[i] = norms[i];
        }
      }
    }

    virtual ~Frame() {
      delete[] codes;

      if (initRgb)
        delete[] initRgb;

      if (initVerts)
        delete[] initVerts;

      if (initNorms)
        delete[] initNorms;
    }

    uint8_t* codes;
    int goodCodes;
    const int id;
    Sophus::SE3d T_wc;
    const int srcTime;
    uint8_t* initRgb;
    Eigen::Vector4f* initVerts;
    Eigen::Vector4f* initNorms;
  };

  std::vector<Frame*> frames;

  const int num;
  std::mt19937 random;
  const int factor;
  const int width;
  const int height;
  const int maxDepth;
  const float photoThresh;
  std::uniform_int_distribution<int32_t> widthDist;
  std::uniform_int_distribution<int32_t> heightDist;
  std::uniform_int_distribution<int32_t> rgbDist;
  std::uniform_int_distribution<int32_t> dDist;

  int lastClosest;
  const uint8_t badCode;
  RGBDOdometry rgbd;

 private:
  void generateFerns();

  float blockHD(const Frame* f1, const Frame* f2);
  float blockHDAware(const Frame* f1, const Frame* f2);

  float photometricCheck(
      const Img<Eigen::Vector4f>& vertSmall,
      const Img<Eigen::Matrix<uint8_t, 3, 1>>& imgSmall,
      const Sophus::SE3d& T_wc_est,
      const Sophus::SE3d& T_wc_fern,
      const uint8_t* fernRgb);

  GPUTexture vertFern;
  GPUTexture vertCurrent;

  GPUTexture normFern;
  GPUTexture normCurrent;

  GPUTexture colorFern;
  GPUTexture colorCurrent;

  Resize resize;

  Img<Eigen::Matrix<uint8_t, 3, 1>> imageBuff;
  Img<Eigen::Vector4f> vertBuff;
  Img<Eigen::Vector4f> normBuff;
};

#endif /* FERNS_H_ */
