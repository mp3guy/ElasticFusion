/*
* This file was written for porting ElasticFusion to windows
* by Filip Srajer (filip.srajer@inf.ethz.ch).
*
*/

#include "Intrinsics.h"

const Intrinsics & Intrinsics::getInstance(float fx,float fy,float cx,float cy)
{
  static const Intrinsics instance(fx,fy,cx,cy);
  return instance;
}

