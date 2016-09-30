/*
 * Written by Filip Srajer.
 *
 */

#include "Intrinsics.h"

const Intrinsics & Intrinsics::getInstance(float fx,float fy,float cx,float cy)
{
  static const Intrinsics instance(fx,fy,cx,cy);
  return instance;
}

