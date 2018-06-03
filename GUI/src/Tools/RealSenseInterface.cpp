#include "RealSenseInterface.h"
#include <functional>
#include <thread>

#ifdef WITH_REALSENSE
RealSenseInterface::RealSenseInterface(int inWidth,int inHeight,int inFps)
    : width(inWidth),
    height(inHeight),
    fps(inFps),
    dev(nullptr),
    initSuccessful(true)
{

    auto list = ctx.query_devices();
    if (list.size() == 0) 
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device tmp_dev = list.front();
    dev = &tmp_dev;
    std::cout << "start" << std::endl;
    //dev->enable_stream(rs2::stream::depth,width,height,rs2::format::z16,fps);
    //dev->enable_stream(rs2::stream::color,width,height,rs2::format::rgb8,fps);

    rs2::pipeline pipe;
    pipe.start();

    latestDepthIndex.assign(-1);
    latestRgbIndex.assign(-1);

    for(int i = 0; i < numBuffers; i++)
    {
        uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
        rgbBuffers[i] = std::pair<uint8_t *,int64_t>(newImage,0);
    }

    for(int i = 0; i < numBuffers; i++)
    {
        uint8_t * newDepth = (uint8_t *)calloc(width * height * 2,sizeof(uint8_t));
        uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
        frameBuffers[i] = std::pair<std::pair<uint8_t *,uint8_t *>,int64_t>(std::pair<uint8_t *,uint8_t *>(newDepth,newImage),0);
    }

    setAutoExposure(true);
    setAutoWhiteBalance(true);

    rgbCallback = new RGBCallback(lastRgbTime,
            latestRgbIndex,
            rgbBuffers);

    depthCallback = new DepthCallback(lastDepthTime,
            latestDepthIndex,
            latestRgbIndex,
            rgbBuffers,
            frameBuffers);
	rs2::frame_queue queue(numBuffers);
	std::thread t([&]() {
    while (true)
    {
        rs2::depth_frame frame;
        if (queue.poll_for_frame(&frame))
        {
	        frame.get_data();
        }
    }
});
t.detach();
//    dev->set_frame_callback(rs2::stream::depth,*depthCallback);
//    dev->set_frame_callback(rs2::stream::color,*rgbCallback);

//    dev->start();
}

RealSenseInterface::~RealSenseInterface()
{
    if(initSuccessful)
    {
        //dev->stop();

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

void RealSenseInterface::setAutoExposure(bool value)
{
//    dev->set_option(rs2::option::color_enable_auto_exposure,value);
//    rs2_set_option(RS2_OPTION_AUTO_EXPOSURE_MODE,)
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
//    dev->set_option(rs2::option::color_enable_auto_white_balance,value);
}

bool RealSenseInterface::getAutoExposure()
{
//    return dev->get_option(rs2::option::color_enable_auto_exposure);
    return false;
}

bool RealSenseInterface::getAutoWhiteBalance()
{
    //return dev->get_option(rs2::option::color_enable_auto_white_balance);
    return false;
}
#else

RealSenseInterface::RealSenseInterface(int inWidth,int inHeight,int inFps)
    : width(inWidth),
    height(inHeight),
    fps(inFps),
    initSuccessful(false)
{
    errorText = "Compiled without Intel RealSense library";
}

RealSenseInterface::~RealSenseInterface()
{
}

void RealSenseInterface::setAutoExposure(bool value)
{
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
}

bool RealSenseInterface::getAutoExposure()
{
    return false;
}

bool RealSenseInterface::getAutoWhiteBalance()
{
    return false;
}
#endif
