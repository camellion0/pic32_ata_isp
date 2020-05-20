/*                      - ATAN2.C -

   The ANSI "atan2" function.

   "atan2" discovers what quadrant the angle is in and calls atan.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"
#include "iccfloat.h"
#include "float.h"
#include "errno.h"



double atan2(double arg1, double arg2)
{
#ifdef _INTRINSIC
  return atan2(arg1, arg2);
#else
  /*======================================================*/
  /* A domain error MAY occure if both arguments are zero */
  /* ANSI 7.5.2.4                                         */
  /*======================================================*/
  if (arg1 == 0.0 && arg2 == 0)
  {
    errno = EDOM;
    return __EDOM_VALUE;
  }

  if ((arg1 + arg2) == arg1)
  {
    if (arg1 >= 0.0)
      return __PIO2;
    else
      return -__PIO2;
  }
  else if (arg2 < 0.0)
  {
    if (arg1 >= 0.0)
      return __PI - __satan(arg1 / arg2);
    else
      return -__PI + __satan(arg1 / arg2);
  }
  else if (arg1 >= 0.0)
    return __satan(arg1 / arg2);
  else
    return -__satan(arg1 / arg2);
#endif
}
