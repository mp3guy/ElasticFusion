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
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_COLOR, inWidth, inHeight, RS2_FORMAT_RGB8,inFps);
    cfg.enable_stream(RS2_STREAM_DEPTH, inWidth, inHeight, RS2_FORMAT_Z16, inFps);

    rs2::device resolve_dev;
    try{
        resolve_dev = cfg.resolve(pipe).get_device();
    }catch(const rs2::error &e){
        initSuccessful = false;
        return;
    }
    dev = &resolve_dev;

    std::cout << "start" << std::endl;
    std::cout << dev->get_info(RS2_CAMERA_INFO_NAME) << " " << dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl; 

    latestDepthIndex.assign(-1);
    latestRgbIndex.assign(-1);

    for (int i = 0; i < numBuffers; i++){
        uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
        rgbBuffers[i] = std::pair<uint8_t *,int64_t>(newImage,0);
    }

    for (int i = 0; i < numBuffers; i++){
        uint8_t * newDepth = (uint8_t *)calloc(width * height * 2,sizeof(uint8_t));
        uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
        frameBuffers[i] = std::pair<std::pair<uint8_t *,uint8_t *>,int64_t>(std::pair<uint8_t *,uint8_t *>(newDepth,newImage),0);
    }

    if (getAutoExposure())
        std::cout << "Auto Exposure enable" << std::endl;
    else
        std::cout << "Auto Exposure disable" << std::endl;
    if(getAutoWhiteBalance())
        std::cout << "Auto White Balance enable" << std::endl;
    else
        std::cout << "Auto White Balance disable" << std::endl;
    setAutoExposure(true);
    setAutoWhiteBalance(true);

	rs2::frame_queue queue(numBuffers);
    std::thread t([&,cfg, inWidth, inHeight, inFps]() {
        pipe.start(cfg);
        rs2::align align(RS2_STREAM_COLOR); //force depth stream align to  color stream
        pipe_active = true;

        while (pipe_active)
        {
            auto frameset = pipe.wait_for_frames();
            auto processed = align.process(frameset);

            rs2::video_frame color_frame = processed.first(RS2_STREAM_COLOR);
            rs2::depth_frame depth_frame = processed.get_depth_frame();

            if (!depth_frame)
            {
                continue;
            }

            lastDepthTime = lastRgbTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
            int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;
            //RGB

            memcpy(rgbBuffers[bufferIndex].first,color_frame.get_data(),
                    color_frame.get_width() * color_frame.get_height() * 3);

            rgbBuffers[bufferIndex].second = lastRgbTime;
            latestRgbIndex++;


            //Depth
            // The multiplication by 2 is here because the depth is actually uint16_t
            memcpy(frameBuffers[bufferIndex].first.first,depth_frame.get_data(),
                    depth_frame.get_width() * depth_frame.get_height() * 2);

            frameBuffers[bufferIndex].second = lastDepthTime;

            int lastImageVal = latestRgbIndex.getValue();

            if(lastImageVal == -1)
            {
                return;
            }

            lastImageVal %= numBuffers;

            memcpy(frameBuffers[bufferIndex].first.second,rgbBuffers[lastImageVal].first,
                    depth_frame.get_width() * depth_frame.get_height() * 3);

            latestDepthIndex++;

        }
        pipe.stop();
    });
    t.detach();
}

RealSenseInterface::~RealSenseInterface()
{
    if(initSuccessful)
    {
        pipe_active = false;

        for(int i = 0; i < numBuffers; i++)
        {
            free(rgbBuffers[i].first);
        }

        for(int i = 0; i < numBuffers; i++)
        {
            free(frameBuffers[i].first.first);
            free(frameBuffers[i].first.second);
        }

    }
}

void RealSenseInterface::setAutoExposure(bool value)
{
    auto sensor = dev->first<rs2::sensor>();
    try {
        sensor.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, value);
    } catch (rs2::invalid_value_error &e) {
        std::cerr << "This camera dose not support Auto White Balance option." << std::endl;
    }
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
    auto sensor = dev->first<rs2::sensor>();
    try {
        return sensor.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, value);
    } catch (rs2::invalid_value_error &e) {
        std::cerr << "This camera does not support Auto White Balance option." << std::endl;
    }
}

bool RealSenseInterface::getAutoExposure()
{
    try {
        auto sensor = dev->first<rs2::sensor>();
        return sensor.get_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE);
    } catch (rs2::invalid_value_error &e) {
        std::cerr << "This camera dose not support Auto White Balance option." << std::endl;
        return false;
    }
}

bool RealSenseInterface::getAutoWhiteBalance()
{
    try {
        auto sensor = dev->first<rs2::sensor>();
        return sensor.get_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
    } catch (rs2::invalid_value_error &e) {
        std::cerr << "This camera does not support Auto White Balance option." << std::endl;
        return false;
    }
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
