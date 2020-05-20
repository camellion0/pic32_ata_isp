/*                      - FABS.C -

   The ANSI "fabs" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"
#include "iccfloat.h"

double fabs(double x)
{
#ifdef _INTRINSIC
  return fabs(x);
#else
  return x < 0.0 ? -x : x;
#endif
}
