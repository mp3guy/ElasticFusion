#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <map>

#ifdef WITH_REALSENSE
#include "librealsense/rs.hpp"
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

#ifdef WITH_REALSENSE
  struct RGBCallback
  {
  public:
    RGBCallback(int64_t & lastRgbTime,
      ThreadMutexObject<int> & latestRgbIndex,
      std::pair<uint8_t *,int64_t> * rgbBuffers)
      : lastRgbTime(lastRgbTime),
      latestRgbIndex(latestRgbIndex),
      rgbBuffers(rgbBuffers)
    {
    }

    void operator()(rs::frame frame)
    {
      lastRgbTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

      int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;

      memcpy(rgbBuffers[bufferIndex].first,frame.get_data(),
        frame.get_width() * frame.get_height() * 3);

      rgbBuffers[bufferIndex].second = lastRgbTime;

      latestRgbIndex++;
    }

  private:
    int64_t & lastRgbTime;
    ThreadMutexObject<int> & latestRgbIndex;
    std::pair<uint8_t *,int64_t> * rgbBuffers;
  };

  struct DepthCallback
  {
  public:
    DepthCallback(int64_t & lastDepthTime,
      ThreadMutexObject<int> & latestDepthIndex,
      ThreadMutexObject<int> & latestRgbIndex,
      std::pair<uint8_t *,int64_t> * rgbBuffers,
      std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> * frameBuffers)
      : lastDepthTime(lastDepthTime),
      latestDepthIndex(latestDepthIndex),
      latestRgbIndex(latestRgbIndex),
      rgbBuffers(rgbBuffers),
      frameBuffers(frameBuffers)
    {
    }

    void operator()(rs::frame frame)
    {
      lastDepthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

      int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;

      // The multiplication by 2 is here because the depth is actually uint16_t
      memcpy(frameBuffers[bufferIndex].first.first,frame.get_data(),
        frame.get_width() * frame.get_height() * 2);

      frameBuffers[bufferIndex].second = lastDepthTime;

      int lastImageVal = latestRgbIndex.getValue();

      if(lastImageVal == -1)
      {
        return;
      }

      lastImageVal %= numBuffers;

      memcpy(frameBuffers[bufferIndex].first.second,rgbBuffers[lastImageVal].first,
        frame.get_width() * frame.get_height() * 3);

      latestDepthIndex++;
    }

  private:
    int64_t & lastDepthTime;
    ThreadMutexObject<int> & latestDepthIndex;
    ThreadMutexObject<int> & latestRgbIndex;

    std::pair<uint8_t *,int64_t> * rgbBuffers;
    std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> * frameBuffers;
  };
#endif

private:
#ifdef WITH_REALSENSE
  rs::device *dev;
  rs::context ctx;

  RGBCallback * rgbCallback;
  DepthCallback * depthCallback;
#endif

  bool initSuccessful;
  std::string errorText;
  
  ThreadMutexObject<int> latestRgbIndex;
  std::pair<uint8_t *,int64_t> rgbBuffers[numBuffers];

  int64_t lastRgbTime;
  int64_t lastDepthTime;

};
