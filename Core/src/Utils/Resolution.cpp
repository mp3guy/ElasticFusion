/*
 * Written by Filip Srajer.
 *
 */

#include "Resolution.h"

const Resolution & Resolution::getInstance(int width,int height)
{
  static const Resolution instance(width,height);
  return instance;
}

