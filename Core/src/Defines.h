/*
 * File written by Filip Srajer, ETH Zurich, CVG.
 *
 */

#pragma once

#if (defined efusion_EXPORTS) && (defined WIN32)
#define EFUSION_API __declspec(dllexport)
#else
#define EFUSION_API __declspec(dllimport)
#endif
