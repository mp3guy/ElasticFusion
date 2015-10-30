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

#include "OpenNI2Interface.h"

OpenNI2Interface::OpenNI2Interface(int inWidth, int inHeight, int fps)
 : width(inWidth),
   height(inHeight),
   fps(fps),
   initSuccessful(true)
{
    //Setup
    openni::Status rc = openni::STATUS_OK;

    const char * deviceURI = openni::ANY_DEVICE;

    rc = openni::OpenNI::initialize();

    std::string errorString(openni::OpenNI::getExtendedError());

    if(errorString.length() > 0)
    {
        errorText.append(errorString);
        initSuccessful = false;
    }
    else
    {
        rc = device.open(deviceURI);
        if (rc != openni::STATUS_OK)
        {
            errorText.append(openni::OpenNI::getExtendedError());
            openni::OpenNI::shutdown();
            initSuccessful = false;
        }
        else
        {
            openni::VideoMode depthMode;
            depthMode.setFps(fps);
            depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
            depthMode.setResolution(width, height);

            openni::VideoMode colorMode;
            colorMode.setFps(fps);
            colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
            colorMode.setResolution(width, height);

            rc = depthStream.create(device, openni::SENSOR_DEPTH);
            if (rc == openni::STATUS_OK)
            {
                depthStream.setVideoMode(depthMode);
                rc = depthStream.start();
                if (rc != openni::STATUS_OK)
                {
                    errorText.append(openni::OpenNI::getExtendedError());
                    depthStream.destroy();
                    initSuccessful = false;
                }
            }
            else
            {
                errorText.append(openni::OpenNI::getExtendedError());
                initSuccessful = false;
            }

            rc = rgbStream.create(device, openni::SENSOR_COLOR);
            if (rc == openni::STATUS_OK)
            {
                rgbStream.setVideoMode(colorMode);
                rc = rgbStream.start();
                if (rc != openni::STATUS_OK)
                {
                    errorText.append(openni::OpenNI::getExtendedError());
                    rgbStream.destroy();
                    initSuccessful = false;
                }
            }
            else
            {
                errorText.append(openni::OpenNI::getExtendedError());
                initSuccessful = false;
            }

            if (!depthStream.isValid() || !rgbStream.isValid())
            {
                errorText.append(openni::OpenNI::getExtendedError());
                openni::OpenNI::shutdown();
                initSuccessful = false;
            }

            if(initSuccessful)
            {
                //For printing out
                formatMap[openni::PIXEL_FORMAT_DEPTH_1_MM] = "1mm";
                formatMap[openni::PIXEL_FORMAT_DEPTH_100_UM] = "100um";
                formatMap[openni::PIXEL_FORMAT_SHIFT_9_2] = "Shift 9 2";
                formatMap[openni::PIXEL_FORMAT_SHIFT_9_3] = "Shift 9 3";

                formatMap[openni::PIXEL_FORMAT_RGB888] = "RGB888";
                formatMap[openni::PIXEL_FORMAT_YUV422] = "YUV422";
                formatMap[openni::PIXEL_FORMAT_GRAY8] = "GRAY8";
                formatMap[openni::PIXEL_FORMAT_GRAY16] = "GRAY16";
                formatMap[openni::PIXEL_FORMAT_JPEG] = "JPEG";

                assert(findMode(width, height, fps) && "Sorry, mode not supported!");

                latestDepthIndex.assign(-1);
                latestRgbIndex.assign(-1);

                for(int i = 0; i < numBuffers; i++)
                {
                    uint8_t * newImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
                    rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newImage, 0);
                }

                for(int i = 0; i < numBuffers; i++)
                {
                    uint8_t * newDepth = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
                    uint8_t * newImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
                    frameBuffers[i] = std::pair<std::pair<uint8_t *, uint8_t *>, int64_t>(std::pair<uint8_t *, uint8_t *>(newDepth, newImage), 0);
                }

                rgbCallback = new RGBCallback(lastRgbTime,
                                              latestRgbIndex,
                                              rgbBuffers);

                depthCallback = new DepthCallback(lastDepthTime,
                                                  latestDepthIndex,
                                                  latestRgbIndex,
                                                  rgbBuffers,
                                                  frameBuffers);

                depthStream.setMirroringEnabled(false);
                rgbStream.setMirroringEnabled(false);

                device.setDepthColorSyncEnabled(true);
                device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

                setAutoExposure(true);
                setAutoWhiteBalance(true);

                rgbStream.addNewFrameListener(rgbCallback);
                depthStream.addNewFrameListener(depthCallback);
            }
        }
    }
}

OpenNI2Interface::~OpenNI2Interface()
{
    if(initSuccessful)
    {
        rgbStream.removeNewFrameListener(rgbCallback);
        depthStream.removeNewFrameListener(depthCallback);

        depthStream.stop();
        rgbStream.stop();
        depthStream.destroy();
        rgbStream.destroy();
        device.close();
        openni::OpenNI::shutdown();

        for(int i = 0; i < numBuffers; i++)
        {
            free(rgbBuffers[i].first);
        }

        for(int i = 0; i < numBuffers; i++)
        {
            free(frameBuffers[i].first.first);
            free(frameBuffers[i].first.second);
        }

        delete rgbCallback;
        delete depthCallback;
    }
}

bool OpenNI2Interface::findMode(int x, int y, int fps)
{
    const openni::Array<openni::VideoMode> & depthModes = depthStream.getSensorInfo().getSupportedVideoModes();

    bool found = false;

    for(int i = 0; i < depthModes.getSize(); i++)
    {
        if(depthModes[i].getResolutionX() == x &&
           depthModes[i].getResolutionY() == y &&
           depthModes[i].getFps() == fps)
        {
            found = true;
            break;
        }
    }

    if(!found)
    {
        return false;
    }

    found = false;

    const openni::Array<openni::VideoMode> & rgbModes = rgbStream.getSensorInfo().getSupportedVideoModes();

    for(int i = 0; i < rgbModes.getSize(); i++)
    {
        if(rgbModes[i].getResolutionX() == x &&
           rgbModes[i].getResolutionY() == y &&
           rgbModes[i].getFps() == fps)
        {
            found = true;
            break;
        }
    }

    return found;
}

void OpenNI2Interface::printModes()
{
    const openni::Array<openni::VideoMode> & depthModes = depthStream.getSensorInfo().getSupportedVideoModes();

    openni::VideoMode currentDepth = depthStream.getVideoMode();

    std::cout << "Depth Modes: (" << currentDepth.getResolutionX() <<
                                     "x" <<
                                     currentDepth.getResolutionY() <<
                                     " @ " <<
                                     currentDepth.getFps() <<
                                     "fps " <<
                                     formatMap[currentDepth.getPixelFormat()] << ")" << std::endl;

    for(int i = 0; i < depthModes.getSize(); i++)
    {
        std::cout << depthModes[i].getResolutionX() <<
                     "x" <<
                     depthModes[i].getResolutionY() <<
                     " @ " <<
                     depthModes[i].getFps() <<
                     "fps " <<
                     formatMap[depthModes[i].getPixelFormat()] << std::endl;
    }

    const openni::Array<openni::VideoMode> & rgbModes = rgbStream.getSensorInfo().getSupportedVideoModes();

    openni::VideoMode currentRGB = depthStream.getVideoMode();

    std::cout << "RGB Modes: (" << currentRGB.getResolutionX() <<
                                   "x" <<
                                   currentRGB.getResolutionY() <<
                                   " @ " <<
                                   currentRGB.getFps() <<
                                   "fps " <<
                                   formatMap[currentRGB.getPixelFormat()] << ")" << std::endl;

    for(int i = 0; i < rgbModes.getSize(); i++)
    {
        std::cout << rgbModes[i].getResolutionX() <<
                     "x" <<
                     rgbModes[i].getResolutionY() <<
                     " @ " <<
                     rgbModes[i].getFps() <<
                     "fps " <<
                     formatMap[rgbModes[i].getPixelFormat()] << std::endl;
    }
}

void OpenNI2Interface::setAutoExposure(bool value)
{
    rgbStream.getCameraSettings()->setAutoExposureEnabled(value);
}

void OpenNI2Interface::setAutoWhiteBalance(bool value)
{
    rgbStream.getCameraSettings()->setAutoWhiteBalanceEnabled(value);
}

bool OpenNI2Interface::getAutoExposure()
{
    return rgbStream.getCameraSettings()->getAutoExposureEnabled();
}

bool OpenNI2Interface::getAutoWhiteBalance()
{
    return rgbStream.getCameraSettings()->getAutoWhiteBalanceEnabled();
}
