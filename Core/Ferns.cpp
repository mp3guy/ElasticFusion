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

#include "Ferns.h"

Ferns::Ferns(int n, int maxDepth, const float photoThresh)
    : num(n),
      factor(8),
      width(Resolution::getInstance().width() / factor),
      height(Resolution::getInstance().height() / factor),
      maxDepth(maxDepth),
      photoThresh(photoThresh),
      widthDist(0, width - 1),
      heightDist(0, height - 1),
      rgbDist(0, 255),
      dDist(400, maxDepth),
      lastClosest(-1),
      badCode(255),
      rgbd(
          Resolution::getInstance().width() / factor,
          Resolution::getInstance().height() / factor,
          Intrinsics::getInstance().cx() / factor,
          Intrinsics::getInstance().cy() / factor,
          Intrinsics::getInstance().fx() / factor,
          Intrinsics::getInstance().fy() / factor),
      vertFern(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false),
      vertCurrent(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false),
      normFern(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false),
      normCurrent(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false),
      colorFern(width, height, GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, false),
      colorCurrent(width, height, GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, false),
      resize(Resolution::getInstance().width(), Resolution::getInstance().height(), width, height),
      imageBuff(width, height),
      vertBuff(width, height),
      normBuff(width, height) {
  random.seed(time(0));
  generateFerns();
}

Ferns::~Ferns() {
  for (size_t i = 0; i < frames.size(); i++) {
    delete frames.at(i);
  }
}

void Ferns::generateFerns() {
  for (int i = 0; i < num; i++) {
    Fern f;

    f.pos(0) = widthDist(random);
    f.pos(1) = heightDist(random);

    f.rgbd(0) = rgbDist(random);
    f.rgbd(1) = rgbDist(random);
    f.rgbd(2) = rgbDist(random);
    f.rgbd(3) = dDist(random);

    conservatory.push_back(f);
  }
}

bool Ferns::addFrame(
    GPUTexture* imageTexture,
    GPUTexture* vertexTexture,
    GPUTexture* normalTexture,
    const Sophus::SE3d& T_wc,
    int srcTime,
    const float threshold) {
  Img<Eigen::Matrix<uint8_t, 3, 1>> img(height, width);
  Img<Eigen::Vector4f> verts(height, width);
  Img<Eigen::Vector4f> norms(height, width);

  resize.image(imageTexture, img);
  resize.vertex(vertexTexture, verts);
  resize.vertex(normalTexture, norms);

  Frame* frame = new Frame(
      num,
      frames.size(),
      T_wc,
      srcTime,
      width * height,
      (uint8_t*)img.data,
      (Eigen::Vector4f*)verts.data,
      (Eigen::Vector4f*)norms.data);

  int* coOccurrences = new int[frames.size()];

  memset(coOccurrences, 0, sizeof(int) * frames.size());

  for (int i = 0; i < num; i++) {
    uint8_t code = badCode;

    if (verts.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0) {
      const Eigen::Matrix<uint8_t, 3, 1>& pix = img.at<Eigen::Matrix<uint8_t, 3, 1>>(
          conservatory.at(i).pos(1), conservatory.at(i).pos(0));

      code = (pix(0) > conservatory.at(i).rgbd(0)) << 3 |
          (pix(1) > conservatory.at(i).rgbd(1)) << 2 | (pix(2) > conservatory.at(i).rgbd(2)) << 1 |
          (int(verts.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) *
               1000.0f) > conservatory.at(i).rgbd(3));

      frame->goodCodes++;

      for (size_t j = 0; j < conservatory.at(i).ids[code].size(); j++) {
        coOccurrences[conservatory.at(i).ids[code].at(j)]++;
      }
    }

    frame->codes[i] = code;
  }

  float minimum = std::numeric_limits<float>::max();

  if (frame->goodCodes > 0) {
    for (size_t i = 0; i < frames.size(); i++) {
      float maxCo = std::min(frame->goodCodes, frames.at(i)->goodCodes);

      float dissim = (float)(maxCo - coOccurrences[i]) / (float)maxCo;

      if (dissim < minimum) {
        minimum = dissim;
      }
    }
  }

  delete[] coOccurrences;

  if ((minimum > threshold || frames.size() == 0) && frame->goodCodes > 0) {
    for (int i = 0; i < num; i++) {
      if (frame->codes[i] != badCode) {
        conservatory.at(i).ids[frame->codes[i]].push_back(frame->id);
      }
    }

    frames.push_back(frame);

    return true;
  } else {
    delete frame;

    return false;
  }
}

Sophus::SE3d Ferns::findFrame(
    std::vector<SurfaceConstraint>& constraints,
    const Sophus::SE3d& T_wc,
    GPUTexture* vertexTexture,
    GPUTexture* normalTexture,
    GPUTexture* imageTexture,
    const int time,
    const bool lost) {
  lastClosest = -1;

  Img<Eigen::Matrix<uint8_t, 3, 1>> imgSmall(height, width);
  Img<Eigen::Vector4f> vertSmall(height, width);
  Img<Eigen::Vector4f> normSmall(height, width);

  resize.image(imageTexture, imgSmall);
  resize.vertex(vertexTexture, vertSmall);
  resize.vertex(normalTexture, normSmall);

  Frame* frame = new Frame(num, 0, Sophus::SE3d(), 0, width * height);

  int* coOccurrences = new int[frames.size()];

  memset(coOccurrences, 0, sizeof(int) * frames.size());

  for (int i = 0; i < num; i++) {
    uint8_t code = badCode;

    if (vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) >
        0) {
      const Eigen::Matrix<uint8_t, 3, 1>& pix = imgSmall.at<Eigen::Matrix<uint8_t, 3, 1>>(
          conservatory.at(i).pos(1), conservatory.at(i).pos(0));

      code = (pix(0) > conservatory.at(i).rgbd(0)) << 3 |
          (pix(1) > conservatory.at(i).rgbd(1)) << 2 | (pix(2) > conservatory.at(i).rgbd(2)) << 1 |
          (int(vertSmall.at<Eigen::Vector4f>(
                   conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) *
               1000.0f) > conservatory.at(i).rgbd(3));

      frame->goodCodes++;

      for (size_t j = 0; j < conservatory.at(i).ids[code].size(); j++) {
        coOccurrences[conservatory.at(i).ids[code].at(j)]++;
      }
    }

    frame->codes[i] = code;
  }

  float minimum = std::numeric_limits<float>::max();
  int minId = -1;

  for (size_t i = 0; i < frames.size(); i++) {
    float maxCo = std::min(frame->goodCodes, frames.at(i)->goodCodes);

    float dissim = (float)(maxCo - coOccurrences[i]) / (float)maxCo;

    if (dissim < minimum && time - frames.at(i)->srcTime > 300) {
      minimum = dissim;
      minId = i;
    }
  }

  delete[] coOccurrences;

  Sophus::SE3d T_wc_est;

  if (minId != -1 && blockHDAware(frame, frames.at(minId)) > 0.3) {
    Sophus::SE3d T_wc_fern = frames.at(minId)->T_wc;

    vertFern.texture->Upload(frames.at(minId)->initVerts, GL_RGBA, GL_FLOAT);
    vertCurrent.texture->Upload(vertSmall.data, GL_RGBA, GL_FLOAT);

    normFern.texture->Upload(frames.at(minId)->initNorms, GL_RGBA, GL_FLOAT);
    normCurrent.texture->Upload(normSmall.data, GL_RGBA, GL_FLOAT);

    //        colorFern.texture->Upload(frames.at(minId)->initRgb, GL_RGB, GL_UNSIGNED_BYTE);
    //        colorCurrent.texture->Upload(imgSmall.data, GL_RGB, GL_UNSIGNED_BYTE);

    // WARNING initICP* must be called before initRGB*
    rgbd.initICPModel(&vertFern, &normFern, T_wc_fern);
    //        rgbd.initRGBModel(&colorFern);

    rgbd.initICP(&vertCurrent, &normCurrent);
    //        rgbd.initRGB(&colorCurrent);

    T_wc_est = T_wc_fern;

    TICK("fernOdom");
    rgbd.getIncrementalTransformation(T_wc_est, false, 100, false, false, false);
    TOCK("fernOdom");

    float photoError =
        photometricCheck(vertSmall, imgSmall, T_wc_est, T_wc_fern, frames.at(minId)->initRgb);

    int icpCountThresh = lost ? 1400 : 2400;

    //        std::cout << rgbd.lastICPError << ", " << rgbd.lastICPCount << ", " << photoError <<
    //        std::endl;

    if (rgbd.lastICPError < 0.0003 && rgbd.lastICPCount > icpCountThresh &&
        photoError < photoThresh) {
      lastClosest = minId;

      for (int i = 0; i < num; i += num / 50) {
        if (vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) >
                0 &&
            int(vertSmall.at<Eigen::Vector4f>(
                    conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) *
                1000.0f) < maxDepth) {
          Eigen::Vector4d worldRawPoint = T_wc *
              Eigen::Vector4d(vertSmall.at<Eigen::Vector4f>(
                                  conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
                              vertSmall.at<Eigen::Vector4f>(
                                  conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
                              vertSmall.at<Eigen::Vector4f>(
                                  conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
                              1.0);

          Eigen::Vector4d worldModelPoint = T_wc_est *
              Eigen::Vector4d(vertSmall.at<Eigen::Vector4f>(
                                  conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
                              vertSmall.at<Eigen::Vector4f>(
                                  conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
                              vertSmall.at<Eigen::Vector4f>(
                                  conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
                              1.0);

          constraints.push_back(SurfaceConstraint(worldRawPoint, worldModelPoint));
        }
      }
    }
  }

  delete frame;

  return T_wc_est;
}

float Ferns::photometricCheck(
    const Img<Eigen::Vector4f>& vertSmall,
    const Img<Eigen::Matrix<uint8_t, 3, 1>>& imgSmall,
    const Sophus::SE3d& T_wc_est,
    const Sophus::SE3d& T_wc_fern,
    const uint8_t* fernRgb) {
  float cx = Intrinsics::getInstance().cx() / factor;
  float cy = Intrinsics::getInstance().cy() / factor;
  float invfx = 1.0f / float(Intrinsics::getInstance().fx() / factor);
  float invfy = 1.0f / float(Intrinsics::getInstance().fy() / factor);

  Img<Eigen::Matrix<uint8_t, 3, 1>> imgFern(height, width, (Eigen::Matrix<uint8_t, 3, 1>*)fernRgb);

  float photoSum = 0;
  int photoCount = 0;

  const Sophus::SE3f T_fern_est_f = (T_wc_fern.inverse() * T_wc_est).cast<float>();

  for (int i = 0; i < num; i++) {
    if (vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) >
            0 &&
        int(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) *
            1000.0f) < maxDepth) {
      Eigen::Vector4f vert_est = Eigen::Vector4f(
          vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
          vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
          vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
          1.0f);

      Eigen::Vector4f vert_fern = T_fern_est_f * vert_est;

      Eigen::Vector2i correspondence_px(
          (vert_fern(0) * (1 / invfx) / vert_fern(2) + cx),
          (vert_fern(1) * (1 / invfy) / vert_fern(2) + cy));

      if (correspondence_px(0) >= 0 && correspondence_px(1) >= 0 && correspondence_px(0) < width &&
          correspondence_px(1) < height &&
          (imgFern.at<Eigen::Matrix<uint8_t, 3, 1>>(correspondence_px(1), correspondence_px(0))(0) >
               0 ||
           imgFern.at<Eigen::Matrix<uint8_t, 3, 1>>(correspondence_px(1), correspondence_px(0))(1) >
               0 ||
           imgFern.at<Eigen::Matrix<uint8_t, 3, 1>>(correspondence_px(1), correspondence_px(0))(2) >
               0)) {
        photoSum +=
            abs((int)imgFern.at<Eigen::Matrix<uint8_t, 3, 1>>(
                    correspondence_px(1), correspondence_px(0))(0) -
                (int)imgSmall.at<Eigen::Matrix<uint8_t, 3, 1>>(
                    conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0));
        photoSum +=
            abs((int)imgFern.at<Eigen::Matrix<uint8_t, 3, 1>>(
                    correspondence_px(1), correspondence_px(0))(1) -
                (int)imgSmall.at<Eigen::Matrix<uint8_t, 3, 1>>(
                    conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1));
        photoSum +=
            abs((int)imgFern.at<Eigen::Matrix<uint8_t, 3, 1>>(
                    correspondence_px(1), correspondence_px(0))(2) -
                (int)imgSmall.at<Eigen::Matrix<uint8_t, 3, 1>>(
                    conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2));
        photoCount++;
      }
    }
  }

  return photoSum / float(photoCount);
}

float Ferns::blockHD(const Frame* f1, const Frame* f2) {
  float sum = 0.0f;

  for (int i = 0; i < num; i++) {
    sum += f1->codes[i] == f2->codes[i];
  }

  sum /= (float)num;

  return sum;
}

float Ferns::blockHDAware(const Frame* f1, const Frame* f2) {
  int count = 0;
  float val = 0;

  for (int i = 0; i < num; i++) {
    if (f1->codes[i] != badCode && f2->codes[i] != badCode) {
      count++;

      if (f1->codes[i] == f2->codes[i]) {
        val += 1.0f;
      }
    }
  }

  return val / (float)count;
}
