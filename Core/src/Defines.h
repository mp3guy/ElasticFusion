/*
* This file was written for porting ElasticFusion to windows
* by Filip Srajer (filip.srajer@inf.ethz.ch).
*
*/

#pragma once

#ifndef WIN32
#  define EFUSION_API
#else
#  ifdef efusion_EXPORTS
#    define EFUSION_API __declspec(dllexport)
#  else
#    define EFUSION_API __declspec(dllimport)
#  endif
#endif
