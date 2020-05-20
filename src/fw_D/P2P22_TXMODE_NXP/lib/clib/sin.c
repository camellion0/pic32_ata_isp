/*                      - SIN.C -

   The ANSI "sin" function.

   Coefficients are #3370 from Hart & Cheney (18.80D).

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"
#include "iccfloat.h"

double sin(double arg)
{
#ifdef _INTRINSIC
  return sin(arg);
#else
  return __sinus(arg,0);
#endif
}
