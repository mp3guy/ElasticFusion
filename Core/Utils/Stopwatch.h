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

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string.h>
#include <sys/time.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#define SEND_INTERVAL_MS 10000

#ifndef DISABLE_STOPWATCH
#define STOPWATCH(name, expression)                                                      \
  do {                                                                                   \
    const uint64_t startTime = Stopwatch::getInstance().getCurrentSystemTime();          \
    expression const uint64_t endTime = Stopwatch::getInstance().getCurrentSystemTime(); \
    Stopwatch::getInstance().addStopwatchTiming(name, endTime - startTime);              \
  } while (false)

#define TICK(name)                                                                        \
  do {                                                                                    \
    Stopwatch::getInstance().tick(name, Stopwatch::getInstance().getCurrentSystemTime()); \
  } while (false)

#define TOCK(name)                                                                        \
  do {                                                                                    \
    Stopwatch::getInstance().tock(name, Stopwatch::getInstance().getCurrentSystemTime()); \
  } while (false)
#else
#define STOPWATCH(name, expression) expression

#define TOCK(name) ((void)0)

#define TICK(name) ((void)0)

#endif

class Stopwatch {
 public:
  static Stopwatch& getInstance() {
    static Stopwatch instance;
    return instance;
  }

  void addStopwatchTiming(std::string name, uint64_t duration) {
    if (duration > 0) {
      timingsMs[name] = (float)(duration) / 1000.0f;
    }
  }

  void setCustomSignature(uint64_t newSignature) {
    signature = newSignature;
  }

  const std::map<std::string, float>& getTimings() {
    return timingsMs;
  }

  void printAll() {
    for (std::map<std::string, float>::const_iterator it = timingsMs.begin(); it != timingsMs.end();
         it++) {
      std::cout << it->first << ": " << it->second << "ms" << std::endl;
    }

    std::cout << std::endl;
  }

  void pulse(std::string name) {
    timingsMs[name] = 1;
  }

  void sendAll() {
    gettimeofday(&clock, 0);

    if ((currentSend = (clock.tv_sec * 1000000 + clock.tv_usec)) - lastSend > SEND_INTERVAL_MS) {
      int size = 0;
      uint8_t* data = serialiseTimings(size);

      sendto(sockfd, data, size, 0, (struct sockaddr*)&servaddr, sizeof(servaddr));

      free(data);

      lastSend = currentSend;
    }
  }

  static uint64_t getCurrentSystemTime() {
    timeval tv;
    gettimeofday(&tv, 0);
    uint64_t time = (uint64_t)(tv.tv_sec * 1000000 + tv.tv_usec);
    return time;
  }

  void tick(std::string name, uint64_t start) {
    ticksUs[name] = start;
  }

  void tock(std::string name, uint64_t end) {
    tocksUs[name] = end;

    const auto tickUs = ticksUs.find(name);

    if (tickUs != ticksUs.end()) {
      const float duration = (float)(end - tickUs->second) / 1000.0f;
      if (duration > 0) {
        timingsMs[name] = duration;
      }
    }
  }

 private:
  Stopwatch() {
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_port = htons(45454);
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    gettimeofday(&clock, 0);

    signature = clock.tv_sec * 1000000 + clock.tv_usec;

    currentSend = lastSend = clock.tv_sec * 1000000 + clock.tv_usec;
  }

  virtual ~Stopwatch() {
    close(sockfd);
  }

  uint8_t* serialiseTimings(int& packetSizeBytes) {
    // Packet structure: [int32_t packetSizeBytes, uint64_t signature, then for each entry]
    packetSizeBytes = sizeof(int) + sizeof(uint64_t);

    // For each entry, length + 1 to include the null terminator for the strings
    auto sumUpMapSizeBytes = [&](const auto& nameValueMap) {
      for (const auto& [name, value] : nameValueMap) {
        packetSizeBytes += sizeof(uint8_t) + (name.length() + 1) + sizeof(decltype(value));
      }
    };

    sumUpMapSizeBytes(timingsMs);
    sumUpMapSizeBytes(ticksUs);
    sumUpMapSizeBytes(tocksUs);

    uint8_t* dataPointer = (uint8_t*)calloc(packetSizeBytes, sizeof(uint8_t));

    // First byte in the packet is the size in bytes of all data
    memcpy(dataPointer, &packetSizeBytes, sizeof(int));

    // Signature unique to each process
    memcpy(dataPointer + sizeof(int), &signature, sizeof(uint64_t));

    uint8_t* interleavedTypeNameValuePointer = dataPointer + sizeof(int) + sizeof(uint64_t);

    auto serializeMap = [&](const uint8_t type, const auto& nameValueMap) {
      for (const auto& [name, value] : nameValueMap) {
        // Write the type of measurement
        memcpy(interleavedTypeNameValuePointer++, &type, sizeof(uint8_t));

        // Write the name
        memcpy(interleavedTypeNameValuePointer, name.c_str(), name.length() + 1);
        interleavedTypeNameValuePointer += name.length() + 1;

        // Write the value
        memcpy(interleavedTypeNameValuePointer, &value, sizeof(decltype(value)));
        interleavedTypeNameValuePointer += sizeof(decltype(value));
      }
    };

    serializeMap(0, timingsMs);
    serializeMap(1, ticksUs);
    serializeMap(2, tocksUs);

    return dataPointer;
  }

  timeval clock;
  long long int currentSend, lastSend;
  uint64_t signature;
  int sockfd;
  struct sockaddr_in servaddr;
  std::map<std::string, float> timingsMs;
  std::map<std::string, uint64_t> tocksUs;
  std::map<std::string, uint64_t> ticksUs;
};
