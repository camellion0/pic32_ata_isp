/*                      - ASIN.C -

   The ANSI "asin" functions.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/


#include "math.h"
#include "float.h"
#include "errno.h"

double asin(double arg)
{
#ifdef _INTRINSIC
  return asin(arg);
#else
  double temp;
  unsigned char sign;

  sign = arg >= 0.0;
  arg = fabs(arg);

  /* Check argument */
  if (arg > 1.0)
  {
    errno = EDOM;
    return __EDOM_VALUE;
  }

  temp = sqrt(1 - arg * arg);
  if (arg > 0.7)
    temp = __PIO2 - atan(temp / arg);
  else
    temp = atan(arg / temp);

  return sign ? temp : -temp;
#endif
}
