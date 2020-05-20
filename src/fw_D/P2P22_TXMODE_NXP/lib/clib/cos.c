/*                      - COS.C -

   The ANSI "cos" function.

   Coefficients are #3370 from Hart & Cheney (18.80D).

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"
#include "iccfloat.h"

double cos(double arg)
{
#ifdef _INTRINSIC
  return cos(arg);
#else
  return __sinus(fabs(arg), 1);
#endif
}
