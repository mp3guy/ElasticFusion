/*
* This file was written for porting ElasticFusion to windows
* by Filip Srajer (filip.srajer@inf.ethz.ch).
*
*/

#include "WindowsExtras.h"

#ifdef WIN32

#include <stdint.h>
#include <string.h>
#define far
#include <Windows.h>
#include <WinSock2.h>

int gettimeofday(struct timeval * tp,struct timezone * tzp)
{
  // Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
  static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

  SYSTEMTIME  system_time;
  FILETIME    file_time;
  uint64_t    time;

  GetSystemTime(&system_time);
  SystemTimeToFileTime(&system_time,&file_time);
  time = ((uint64_t)file_time.dwLowDateTime);
  time += ((uint64_t)file_time.dwHighDateTime) << 32;

  tp->tv_sec = (long)((time - EPOCH) / 10000000L);
  tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
  return 0;
}

void *mempcpy(void *dest,const void *src,size_t n)
{
  return (char *)memcpy(dest,src,n) + n;
}

#endif // WIN32