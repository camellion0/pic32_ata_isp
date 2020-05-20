/*                      - COSH.C -

   The ANSI "cosh" function.


   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "iccfloat.h"
#include "float.h"
#include "errno.h"

double cosh(double arg)
{
#ifdef _INTRINSIC
  return cosh(arg);
#else
  double res;

  arg = fabs(arg);
  res = 0.5 * exp(arg);
#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
  if (arg < 7.98)
#else
  if (arg < 15.95)
#endif
    return (res + 0.25 / res);
  else if (arg < 2 * DBL_MAX_EXP * __LOG2)    /* 2 * MAX_EXP * LN 2*/
    return res;

  errno = ERANGE;
  return HUGE_VAL;
#endif
}
