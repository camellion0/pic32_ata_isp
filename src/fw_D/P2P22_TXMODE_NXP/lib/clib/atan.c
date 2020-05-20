/*                      - ATAN.C -

   The ANSI "atan" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"
#include "iccfloat.h"

/*
  atan makes its argument positive and
  calls the inner routine satan.
*/

double atan(double arg)
{
#ifdef _INTRINSIC
  return atan(arg);
#else
  if (arg >= 0.0)
    return __satan(arg);
  else
    return -__satan(arg);
#endif
}
