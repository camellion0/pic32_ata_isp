/*                      - SINH.C -

   The ANSI "sinh" function.


   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "iccfloat.h"
#include "float.h"
#include "errno.h"


double sinh(double arg)
{
#ifdef _INTRINSIC
  return sinh(arg);
#else
  unsigned char sign;

  sign = arg < 0.0;
  arg = fabs(arg);

  if (arg > 2 * DBL_MAX_EXP * __LOG2)         /* 2 * MAX_EXP * LN 2*/
  {
    errno = ERANGE;
    arg = HUGE_VAL;
  }
#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
  else if (arg < 5.97E-8)
#else
  else if (arg < 5.552E-17)
#endif
    ;

#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
  else if (arg < 7.98)
#else
  else if (arg < 15.95)
#endif
  {
    arg = 0.5 * exp(arg);
    arg = arg - 0.25 / arg;
  }
  return sign ? -arg : arg;
#endif
}
