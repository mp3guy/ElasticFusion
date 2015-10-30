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
 
#include <ElasticFusion.h>
#include <Utils/RGBDOdometry.h>

#include <string>
#include <iomanip>
#include <fstream>

std::ifstream asFile;
std::string directory;
Eigen::Matrix3f K;

void loadImage(GPUTexture & image, const std::string & name)
{
    std::string imageLoc = directory;
    imageLoc.append(name);

    pangolin::TypedImage img = pangolin::LoadImage(imageLoc);

    Img<Eigen::Matrix<unsigned char, 3, 1>> imageRaw(480, 640, (Eigen::Matrix<unsigned char, 3, 1> *)img.ptr);

    image.texture->Upload(imageRaw.data, GL_RGB, GL_UNSIGNED_BYTE);
}

void loadDepth(GPUTexture & depth, const std::string & name)
{
    std::string depthLoc = directory;
    depthLoc.append(name);

    pangolin::TypedImage img = pangolin::LoadImage(depthLoc);

    Img<unsigned short> depthRaw(480, 640, (unsigned short *)img.ptr);

    for(unsigned int i = 0; i < 480; i++)
    {
        for(unsigned int j = 0; j < 640; j++)
        {
            depthRaw.at<unsigned short>(i, j) /= 5;
        }
    }

    depth.texture->Upload(depthRaw.data, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT);
}

Eigen::Vector3f getVertex(const int row, const int col, const float depth)
{
    return Eigen::Vector3f((col - K(0, 2)) * depth * (1.f / K(0, 0)),
                           (row - K(1, 2)) * depth * (1.f / K(1, 1)),
                           depth);
}

void loadVertices(GPUTexture & vertices, GPUTexture & normals, const std::string & name)
{
    std::string depthLoc = directory;
    depthLoc.append(name);

    pangolin::TypedImage img = pangolin::LoadImage(depthLoc);

    Img<unsigned short> depthRaw(480, 640, (unsigned short *)img.ptr);

    Img<Eigen::Vector4f> verts(480, 640);
    Img<Eigen::Vector4f> norms(480, 640);

    Eigen::Vector4f point(0, 0, 0, 1);
    Eigen::Vector4f norm(0, 0, 0, 1);

    for(unsigned int row = 1; row < 480 - 1; row++)
    {
        for(unsigned int col = 1; col < 640 - 1; col++)
        {
            if(depthRaw.at<unsigned short>(row, col) > 0 &&
               depthRaw.at<unsigned short>(row + 1, col) > 0 &&
               depthRaw.at<unsigned short>(row, col + 1) > 0 &&
               depthRaw.at<unsigned short>(row - 1, col) > 0 &&
               depthRaw.at<unsigned short>(row, col - 1) > 0)
            {
                Eigen::Vector3f actual = getVertex(row, col, depthRaw.at<unsigned short>(row, col) / 5000.f);

                Eigen::Vector3f fore = getVertex(row, col + 1, depthRaw.at<unsigned short>(row, col + 1) / 5000.f);

                Eigen::Vector3f del_x = fore - actual;

                fore = getVertex(row + 1, col, depthRaw.at<unsigned short>(row + 1, col) / 5000.f);

                Eigen::Vector3f del_y = fore - actual;

                Eigen::Vector3f normal = (del_x.cross(del_y)).normalized();

                point.head(3) = actual;
                norm.head(3) = normal;
            }
            else
            {
                point = Eigen::Vector4f(0, 0, 0, 1);
                norm = Eigen::Vector4f(0, 0, 0, 1);
            }

            verts.at<Eigen::Vector4f>(row, col)(0) = point(0);
            verts.at<Eigen::Vector4f>(row, col)(1) = point(1);
            verts.at<Eigen::Vector4f>(row, col)(2) = point(2);
            verts.at<Eigen::Vector4f>(row, col)(3) = point(3);

            norms.at<Eigen::Vector4f>(row, col)(0) = norm(0);
            norms.at<Eigen::Vector4f>(row, col)(1) = norm(1);
            norms.at<Eigen::Vector4f>(row, col)(2) = norm(2);
            norms.at<Eigen::Vector4f>(row, col)(3) = norm(3);
        }
    }

    vertices.texture->Upload(verts.data, GL_RGBA, GL_FLOAT);
    normals.texture->Upload(norms.data, GL_RGBA, GL_FLOAT);
}

void display(const GPUTexture & firstImage, const GPUTexture & secondImage, const int counter)
{
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pangolin::Display("Image").Activate();
    if(counter % 2 == 0)
        firstImage.texture->RenderToViewport(true);
    else
        secondImage.texture->RenderToViewport(true);

    pangolin::FinishFrame();
    glFinish();
}

int main(int argc, char * argv[])
{
    Stopwatch::getInstance().setCustomSignature(123412);

    K << 528,   0, 320,
           0, 528, 240,
           0,   0,   1;

    assert(argc == 2 && "Please supply the folder containing 1c.png, 1d.png, 2c.png and 2d.png");

    directory.append(argv[1]);

    if(directory.at(directory.size() - 1) != '/')
    {
        directory.append("/");
    }

    pangolin::CreateWindowAndBind("GPUTest", 640, 480);
    pangolin::Display("Image").SetAspect(640.0f / 480.0f);

    GPUTexture firstImage(640, 480,
                          GL_RGBA,
                          GL_RGB,
                          GL_UNSIGNED_BYTE,
                          false, true);

    loadImage(firstImage, "1c.png");

    GPUTexture secondImage(640, 480,
                           GL_RGBA,
                           GL_RGB,
                           GL_UNSIGNED_BYTE,
                           false, true);

    loadImage(secondImage, "2c.png");

    display(firstImage, secondImage, 0);

    GPUTexture secondDepth(640, 480,
                           GL_LUMINANCE16UI_EXT,
                           GL_LUMINANCE_INTEGER_EXT,
                           GL_UNSIGNED_SHORT,
                           false,
                           true);

    loadDepth(secondDepth, "2d.png");

    GPUTexture vertexTexture(640, 480,
                             GL_RGBA32F,
                             GL_LUMINANCE,
                             GL_FLOAT,
                             false, true);

    GPUTexture normalTexture(640, 480,
                             GL_RGBA32F,
                             GL_LUMINANCE,
                             GL_FLOAT,
                             false, true);

    loadVertices(vertexTexture, normalTexture, "1d.png");

    cudaDeviceProp prop;

    cudaGetDeviceProperties(&prop, 0);

    std::string dev(prop.name);

    std::cout << dev << std::endl;

    RGBDOdometry odom(640, 480, K(0, 2), K(1, 2), K(0, 0), K(1, 1));

    Eigen::Matrix4f currPose = Eigen::Matrix4f::Identity();

    float icpStepMeanTime = std::numeric_limits<float>::max();
    float rgbStepMeanTime = std::numeric_limits<float>::max();
    float rgbResMeanTime = std::numeric_limits<float>::max();
    float so3StepMeanTime = std::numeric_limits<float>::max();
    int count = 0;

    int threads = 16;
    int blocks = 16;

    int icpStepBestThreads = threads;
    int rgbStepBestThreads = threads;
    int rgbResBestThreads = threads;
    int so3StepBestThreads = threads;

    int icpStepBestBlocks = blocks;
    int rgbStepBestBlocks = blocks;
    int rgbResBestBlocks = blocks;
    int so3StepBestBlocks = blocks;

    float icpStepBestFast = icpStepMeanTime;
    float rgbStepBestFast = rgbStepMeanTime;
    float rgbResBestFast = rgbResMeanTime;
    float so3StepBestFast = so3StepMeanTime;

    std::cout << "Searching for the best thread/block configuration for your GPU..." << std::endl;

    float counter = 0;

    for(threads = 16; threads <= 512 && !pangolin::ShouldQuit(); threads += 16)
    {
        for(blocks = 16; blocks <= 512 && !pangolin::ShouldQuit(); blocks += 16)
        {
            icpStepMeanTime = 0.0f;
            rgbStepMeanTime = 0.0f;
            rgbResMeanTime = 0.0f;
            so3StepMeanTime = 0.0f;

            count = 0;

            for(int i = 0; i < 5 && !pangolin::ShouldQuit(); i++)
            {
                odom.initICPModel(&vertexTexture, &normalTexture, 20.0f, currPose);
                odom.initICP(&secondDepth, 20.0f);

                Eigen::Vector3f trans = currPose.topRightCorner(3, 1);
                Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = currPose.topLeftCorner(3, 3);

                GPUConfig::getInstance().icpStepThreads = threads;
                GPUConfig::getInstance().icpStepBlocks = blocks;

                GPUConfig::getInstance().rgbStepThreads = threads;
                GPUConfig::getInstance().rgbStepBlocks = blocks;

                GPUConfig::getInstance().rgbResThreads = threads;
                GPUConfig::getInstance().rgbResBlocks = blocks;

                GPUConfig::getInstance().so3StepThreads = threads;
                GPUConfig::getInstance().so3StepBlocks = blocks;

                odom.getIncrementalTransformation(trans, rot, false, 10, false, false, true);

                icpStepMeanTime = (float(count) * icpStepMeanTime + (Stopwatch::getInstance().getTimings().at("icpStep") * 10)) / float(count + 1);
                rgbStepMeanTime = (float(count) * rgbStepMeanTime + (Stopwatch::getInstance().getTimings().at("rgbStep") * 10)) / float(count + 1);
                rgbResMeanTime = (float(count) * rgbResMeanTime + (Stopwatch::getInstance().getTimings().at("computeRgbResidual") * 10)) / float(count + 1);
                so3StepMeanTime = (float(count) * so3StepMeanTime + (Stopwatch::getInstance().getTimings().at("so3Step") * 10)) / float(count + 1);

                count++;
            }

            counter++;

            if(icpStepMeanTime < icpStepBestFast)
            {
                icpStepBestFast = icpStepMeanTime;
                icpStepBestThreads = threads;
                icpStepBestBlocks = blocks;
            }

            if(rgbStepMeanTime < rgbStepBestFast)
            {
                rgbStepBestFast = rgbStepMeanTime;
                rgbStepBestThreads = threads;
                rgbStepBestBlocks = blocks;
            }

            if(rgbResMeanTime < rgbResBestFast)
            {
                rgbResBestFast = rgbResMeanTime;
                rgbResBestThreads = threads;
                rgbResBestBlocks = blocks;
            }

            if(so3StepMeanTime < so3StepBestFast)
            {
                so3StepBestFast = so3StepMeanTime;
                so3StepBestThreads = threads;
                so3StepBestBlocks = blocks;
            }

            std::cout << "\rBest: " << (so3StepBestFast + rgbResBestFast + rgbStepBestFast + icpStepBestFast) << ", " << int((counter / 1024.f) * 100.f) << "%    "; std::cout.flush();

            Stopwatch::getInstance().sendAll();

            display(firstImage, secondImage, counter);
        }
    }

    std::cout << std::endl;

    std::cout << "icpStepMap[\"" << dev << "\"] = std::pair<int, int>(" << icpStepBestThreads <<", " << icpStepBestBlocks << ");" << std::endl;
    std::cout << "rgbStepMap[\"" << dev << "\"] = std::pair<int, int>(" << rgbStepBestThreads <<", " << rgbStepBestBlocks << ");" << std::endl;
    std::cout << "rgbResMap[\"" << dev << "\"] = std::pair<int, int>(" << rgbResBestThreads <<", " << rgbResBestBlocks << ");" << std::endl;
    std::cout << "so3StepMap[\"" << dev << "\"] = std::pair<int, int>(" << so3StepBestThreads <<", " << so3StepBestBlocks << ");" << std::endl;
}

