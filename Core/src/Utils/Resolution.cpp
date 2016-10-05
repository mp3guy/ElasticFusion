/*
* This file was written for porting ElasticFusion to windows
* by Filip Srajer (filip.srajer@inf.ethz.ch).
*
*/

#include "Resolution.h"

const Resolution & Resolution::getInstance(int width,int height)
{
  static const Resolution instance(width,height);
  return instance;
}

