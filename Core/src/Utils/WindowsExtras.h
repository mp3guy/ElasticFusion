/*
* This file was written for porting ElasticFusion to windows
* by Filip Srajer (filip.srajer@inf.ethz.ch).
* 
*/

#pragma once

#include "../Defines.h"

#ifdef WIN32

EFUSION_API int gettimeofday(struct timeval * tp,struct timezone * tzp);

EFUSION_API void *mempcpy(void *dest,const void *src,size_t n);

#endif // WIN32