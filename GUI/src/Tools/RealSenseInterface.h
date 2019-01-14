#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <map>
#include <cstring>
#include <cstdio>
#include <atomic>
#ifdef WITH_REALSENSE
#include "librealsense2/rs.hpp"
#endif

#include "ThreadMutexObject.h"
#include "CameraInterface.h"

class RealSenseInterface : public CameraInterface
{
    public:
        RealSenseInterface(int width = 640,int height = 480,int fps = 30);
        virtual ~RealSenseInterface();

        const int width,height,fps;

        bool getAutoExposure();
        bool getAutoWhiteBalance();
        virtual void setAutoExposure(bool value);
        virtual void setAutoWhiteBalance(bool value);

        virtual bool ok()
        {
            return initSuccessful;
        }

        virtual std::string error()
        {
            return errorText;
        }

    private:
#ifdef WITH_REALSENSE
        rs2::device *dev;
        rs2::context ctx;
        rs2::pipeline pipe;
        std::atomic<bool> pipe_active;
#endif

        bool initSuccessful;
        std::string errorText;

        ThreadMutexObject<int> latestRgbIndex;
        std::pair<uint8_t *,int64_t> rgbBuffers[numBuffers];

        int64_t lastRgbTime;
        int64_t lastDepthTime;

};
