/*
* This file was written for porting ElasticFusion to windows
* by Filip Srajer (filip.srajer@inf.ethz.ch).
*
*/

#pragma once

#if (defined efusion_EXPORTS) && (defined WIN32)
#define EFUSION_API __declspec(dllexport)
#else
#define EFUSION_API __declspec(dllimport)
#endif
