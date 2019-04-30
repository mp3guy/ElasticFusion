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
   rgbd(Resolution::getInstance().width() / factor,
        Resolution::getInstance().height() / factor,
        Intrinsics::getInstance().cx() / factor,
        Intrinsics::getInstance().cy() / factor,
        Intrinsics::getInstance().fx() / factor,
        Intrinsics::getInstance().fy() / factor),
   vertFern(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false, true),
   vertCurrent(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false, true),
   normFern(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false, true),
   normCurrent(width, height, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT, false, true),
   colorFern(width, height, GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, false, true),
   colorCurrent(width, height, GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, false, true),
   resize(Resolution::getInstance().width(), Resolution::getInstance().height(), width, height),
   imageBuff(width, height),
   vertBuff(width, height),
   normBuff(width, height)
{
    random.seed(time(0));
    generateFerns();
}

Ferns::~Ferns()
{
    for(size_t i = 0; i < frames.size(); i++)
    {
        delete frames.at(i);
    }
}

void Ferns::generateFerns()
{
    for(int i = 0; i < num; i++)
    {
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

bool Ferns::addFrame(GPUTexture * imageTexture, GPUTexture * vertexTexture, GPUTexture * normalTexture, const Eigen::Matrix4f & pose, int srcTime, const float threshold)
{
    Img<Eigen::Matrix<unsigned char, 3, 1>> img(height, width);
    Img<Eigen::Vector4f> verts(height, width);
    Img<Eigen::Vector4f> norms(height, width);

    resize.image(imageTexture, img);
    resize.vertex(vertexTexture, verts);
    resize.vertex(normalTexture, norms);

    Frame * frame = new Frame(num,
                              frames.size(),
                              pose,
                              srcTime,
                              width * height,
                              (unsigned char *)img.data,
                              (Eigen::Vector4f *)verts.data,
                              (Eigen::Vector4f *)norms.data);

    int * coOccurrences = new int[frames.size()];

    memset(coOccurrences, 0, sizeof(int) * frames.size());

    for(int i = 0; i < num; i++)
    {
        unsigned char code = badCode;

        if(verts.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0)
        {
            const Eigen::Matrix<unsigned char, 3, 1> & pix = img.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0));

            code = (pix(0) > conservatory.at(i).rgbd(0)) << 3 |
                   (pix(1) > conservatory.at(i).rgbd(1)) << 2 |
                   (pix(2) > conservatory.at(i).rgbd(2)) << 1 |
                   (int(verts.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) * 1000.0f) > conservatory.at(i).rgbd(3));

            frame->goodCodes++;

            for(size_t j = 0; j < conservatory.at(i).ids[code].size(); j++)
            {
                coOccurrences[conservatory.at(i).ids[code].at(j)]++;
            }
        }

        frame->codes[i] = code;
    }

    float minimum = std::numeric_limits<float>::max();

    if(frame->goodCodes > 0)
    {
        for(size_t i = 0; i < frames.size(); i++)
        {
            float maxCo = std::min(frame->goodCodes, frames.at(i)->goodCodes);

            float dissim = (float)(maxCo - coOccurrences[i]) / (float)maxCo;

            if(dissim < minimum)
            {
                minimum = dissim;
            }
        }
    }

    delete [] coOccurrences;

    if((minimum > threshold || frames.size() == 0) && frame->goodCodes > 0)
    {
        for(int i = 0; i < num; i++)
        {
            if(frame->codes[i] != badCode)
            {
                conservatory.at(i).ids[frame->codes[i]].push_back(frame->id);
            }
        }

        frames.push_back(frame);

        return true;
    }
    else
    {
        delete frame;

        return false;
    }
}

Eigen::Matrix4f Ferns::findFrame(std::vector<SurfaceConstraint> & constraints,
                                 const Eigen::Matrix4f & currPose,
                                 GPUTexture * vertexTexture,
                                 GPUTexture * normalTexture,
                                 GPUTexture * imageTexture,
                                 const int time,
                                 const bool lost)
{
    lastClosest = -1;

    Img<Eigen::Matrix<unsigned char, 3, 1>> imgSmall(height, width);
    Img<Eigen::Vector4f> vertSmall(height, width);
    Img<Eigen::Vector4f> normSmall(height, width);

    resize.image(imageTexture, imgSmall);
    resize.vertex(vertexTexture, vertSmall);
    resize.vertex(normalTexture, normSmall);

    Frame * frame = new Frame(num, 0, Eigen::Matrix4f::Identity(), 0, width * height);

    int * coOccurrences = new int[frames.size()];

    memset(coOccurrences, 0, sizeof(int) * frames.size());

    for(int i = 0; i < num; i++)
    {
        unsigned char code = badCode;

        if(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0)
        {
            const Eigen::Matrix<unsigned char, 3, 1> & pix = imgSmall.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0));

            code = (pix(0) > conservatory.at(i).rgbd(0)) << 3 |
                   (pix(1) > conservatory.at(i).rgbd(1)) << 2 |
                   (pix(2) > conservatory.at(i).rgbd(2)) << 1 |
                   (int(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) * 1000.0f) > conservatory.at(i).rgbd(3));

            frame->goodCodes++;

            for(size_t j = 0; j < conservatory.at(i).ids[code].size(); j++)
            {
                coOccurrences[conservatory.at(i).ids[code].at(j)]++;
            }
        }

        frame->codes[i] = code;
    }

    float minimum = std::numeric_limits<float>::max();
    int minId = -1;

    for(size_t i = 0; i < frames.size(); i++)
    {
        float maxCo = std::min(frame->goodCodes, frames.at(i)->goodCodes);

        float dissim = (float)(maxCo - coOccurrences[i]) / (float)maxCo;

        if(dissim < minimum && time - frames.at(i)->srcTime > 300)
        {
            minimum = dissim;
            minId = i;
        }
    }

    delete [] coOccurrences;

    Eigen::Matrix4f estPose = Eigen::Matrix4f::Identity();

    if(minId != -1 && blockHDAware(frame, frames.at(minId)) > 0.3)
    {
        Eigen::Matrix4f fernPose = frames.at(minId)->pose;

        vertFern.texture->Upload(frames.at(minId)->initVerts, GL_RGBA, GL_FLOAT);
        vertCurrent.texture->Upload(vertSmall.data, GL_RGBA, GL_FLOAT);

        normFern.texture->Upload(frames.at(minId)->initNorms, GL_RGBA, GL_FLOAT);
        normCurrent.texture->Upload(normSmall.data, GL_RGBA, GL_FLOAT);

//        colorFern.texture->Upload(frames.at(minId)->initRgb, GL_RGB, GL_UNSIGNED_BYTE);
//        colorCurrent.texture->Upload(imgSmall.data, GL_RGB, GL_UNSIGNED_BYTE);

        //WARNING initICP* must be called before initRGB*
        rgbd.initICPModel(&vertFern, &normFern, fernPose);
//        rgbd.initRGBModel(&colorFern);

        rgbd.initICP(&vertCurrent, &normCurrent);
//        rgbd.initRGB(&colorCurrent);

        Eigen::Vector3f trans = fernPose.topRightCorner(3, 1);
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = fernPose.topLeftCorner(3, 3);

        TICK("fernOdom");
        rgbd.getIncrementalTransformation(trans,
                                          rot,
                                          false,
                                          100,
                                          false,
                                          false,
                                          false);
        TOCK("fernOdom");

        estPose.topRightCorner(3, 1) = trans;
        estPose.topLeftCorner(3, 3) = rot;

        float photoError = photometricCheck(vertSmall, imgSmall, estPose, fernPose, frames.at(minId)->initRgb);

        int icpCountThresh = lost ? 1400 : 2400;

//        std::cout << rgbd.lastICPError << ", " << rgbd.lastICPCount << ", " << photoError << std::endl;

        if(rgbd.lastICPError < 0.0003 && rgbd.lastICPCount > icpCountThresh && photoError < photoThresh)
        {
            lastClosest = minId;

            for(int i = 0; i < num; i += num / 50)
            {
                if(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0 &&
                   int(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) * 1000.0f) < maxDepth)
                {
                    Eigen::Vector4f worldRawPoint = currPose * Eigen::Vector4f(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
                                                                               vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
                                                                               vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
                                                                               1.0f);

                    Eigen::Vector4f worldModelPoint = estPose * Eigen::Vector4f(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
                                                                                vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
                                                                                vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
                                                                                1.0f);

                    constraints.push_back(SurfaceConstraint(worldRawPoint, worldModelPoint));
                }
            }
        }
    }

    delete frame;

    return estPose;
}

float Ferns::photometricCheck(const Img<Eigen::Vector4f> & vertSmall,
                              const Img<Eigen::Matrix<unsigned char, 3, 1>> & imgSmall,
                              const Eigen::Matrix4f & estPose,
                              const Eigen::Matrix4f & fernPose,
                              const unsigned char * fernRgb)
{
    float cx = Intrinsics::getInstance().cx() / factor;
    float cy = Intrinsics::getInstance().cy() / factor;
    float invfx = 1.0f / float(Intrinsics::getInstance().fx() / factor);
    float invfy = 1.0f / float(Intrinsics::getInstance().fy() / factor);

    Img<Eigen::Matrix<unsigned char, 3, 1>> imgFern(height, width, (Eigen::Matrix<unsigned char, 3, 1> *)fernRgb);

    float photoSum = 0;
    int photoCount = 0;

    for(int i = 0; i < num; i++)
    {
        if(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) > 0 &&
           int(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2) * 1000.0f) < maxDepth)
        {
            Eigen::Vector4f vertPoint = Eigen::Vector4f(vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0),
                                                        vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1),
                                                        vertSmall.at<Eigen::Vector4f>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2),
                                                        1.0f);

            Eigen::Matrix4f diff = fernPose.inverse() * estPose;

            Eigen::Vector4f worldCorrPoint = diff * vertPoint;

            Eigen::Vector2i correspondence((worldCorrPoint(0) * (1/invfx) / worldCorrPoint(2) + cx), (worldCorrPoint(1) * (1/invfy) / worldCorrPoint(2) + cy));

            if(correspondence(0) >= 0 && correspondence(1) >= 0 && correspondence(0) < width && correspondence(1) < height &&
               (imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(0) > 0 ||
                imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(1) > 0 ||
                imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(2) > 0))
            {
                photoSum += abs((int)imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(0) - (int)imgSmall.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(0));
                photoSum += abs((int)imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(1) - (int)imgSmall.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(1));
                photoSum += abs((int)imgFern.at<Eigen::Matrix<unsigned char, 3, 1>>(correspondence(1), correspondence(0))(2) - (int)imgSmall.at<Eigen::Matrix<unsigned char, 3, 1>>(conservatory.at(i).pos(1), conservatory.at(i).pos(0))(2));
                photoCount++;
            }
        }
    }

    return photoSum / float(photoCount);
}

float Ferns::blockHD(const Frame * f1, const Frame * f2)
{
    float sum = 0.0f;

    for(int i = 0; i < num; i++)
    {
        sum += f1->codes[i] == f2->codes[i];
    }

    sum /= (float)num;

    return sum;
}

float Ferns::blockHDAware(const Frame * f1, const Frame * f2)
{
    int count = 0;
    float val = 0;

    for(int i = 0; i < num; i++)
    {
        if(f1->codes[i] != badCode && f2->codes[i] != badCode)
        {
            count++;

            if(f1->codes[i] == f2->codes[i])
            {
                val += 1.0f;
            }
        }
    }

    return val / (float)count;
}
