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
 */

#ifndef FERNS_H_
#define FERNS_H_

#include <random>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>
#include <limits>

#include "Utils/Resolution.h"
#include "Utils/Intrinsics.h"
#include "Utils/RGBDOdometry.h"
#include "Shaders/Resize.h"

class Ferns
{
    public:
        Ferns(int n, int maxDepth, const float photoThresh);
        virtual ~Ferns();

        bool addFrame(GPUTexture * imageTexture, GPUTexture * vertexTexture, GPUTexture * normalTexture, const Eigen::Matrix4f & pose, int srcTime, const float threshold);

        class SurfaceConstraint
        {
            public:
                SurfaceConstraint(const Eigen::Vector4f & sourcePoint,
                                  const Eigen::Vector4f & targetPoint)
                 : sourcePoint(sourcePoint),
                   targetPoint(targetPoint)
                {}

                Eigen::Vector4f sourcePoint;
                Eigen::Vector4f targetPoint;
        };

        Eigen::Matrix4f findFrame(std::vector<SurfaceConstraint> & constraints,
                                  const Eigen::Matrix4f & currPose,
                                  GPUTexture * vertexTexture,
                                  GPUTexture * normalTexture,
                                  GPUTexture * imageTexture,
                                  const int time,
                                  const bool lost);

        class Fern
        {
            public:
                Fern()
                {}

                Eigen::Vector2i pos;
                Eigen::Vector4i rgbd;
                std::vector<int> ids[16];
        };

        std::vector<Fern> conservatory;

        class Frame
        {
            public:
                Frame(int n,
                      int id,
                      const Eigen::Matrix4f & pose,
                      const int srcTime,
                      const int numPixels,
                      unsigned char * rgb = 0,
                      Eigen::Vector4f * verts = 0,
                      Eigen::Vector4f * norms = 0)
                 : goodCodes(0),
                   id(id),
                   pose(pose),
                   srcTime(srcTime),
                   initRgb(rgb),
                   initVerts(verts),
                   initNorms(norms)
                {
                    codes = new unsigned char[n];

                    if(rgb)
                    {
                        this->initRgb = new unsigned char[numPixels * 3];
                        memcpy(this->initRgb, rgb, numPixels * 3);
                    }

                    if(verts)
                    {
                        this->initVerts = new Eigen::Vector4f[numPixels];
                        memcpy(this->initVerts, verts, numPixels * sizeof(Eigen::Vector4f));
                    }

                    if(norms)
                    {
                        this->initNorms = new Eigen::Vector4f[numPixels];
                        memcpy(this->initNorms, norms, numPixels * sizeof(Eigen::Vector4f));
                    }
                }

                virtual ~Frame()
                {
                    delete [] codes;

                    if(initRgb)
                        delete [] initRgb;

                    if(initVerts)
                        delete [] initVerts;

                    if(initNorms)
                        delete [] initNorms;
                }

                unsigned char * codes;
                int goodCodes;
                const int id;
                Eigen::Matrix4f pose;
                const int srcTime;
                unsigned char * initRgb;
                Eigen::Vector4f * initVerts;
                Eigen::Vector4f * initNorms;
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
        const unsigned char badCode;
        RGBDOdometry rgbd;

    private:
        void generateFerns();

        float blockHD(const Frame * f1, const Frame * f2);
        float blockHDAware(const Frame * f1, const Frame * f2);

        float photometricCheck(const Img<Eigen::Vector4f> & vertSmall,
                               const Img<Eigen::Matrix<unsigned char, 3, 1>> & imgSmall,
                               const Eigen::Matrix4f & estPose,
                               const Eigen::Matrix4f & fernPose,
                               const unsigned char * fernRgb);

        GPUTexture vertFern;
        GPUTexture vertCurrent;

        GPUTexture normFern;
        GPUTexture normCurrent;

        GPUTexture colorFern;
        GPUTexture colorCurrent;

        Resize resize;

        Img<Eigen::Matrix<unsigned char, 3, 1>> imageBuff;
        Img<Eigen::Vector4f> vertBuff;
        Img<Eigen::Vector4f> normBuff;
};

#endif /* FERNS_H_ */
