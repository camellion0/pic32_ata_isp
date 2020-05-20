/*                      - FMOD.C -

   The ANSI "fmod" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"

double fmod(double x, double y)
{
#ifdef _INTRINSIC
  return fmod(x, y);
#else
  double res;

  if (y == 0.0)
  {
    /* Can make a an EDOM here */
    /* errno = EDOM; */
    /* return (__EDOM_VALUE); */
    return 0.0;
  }

  res = x / y;
  if (res < 0.0)
    res = -floor(fabs(res));
  else
    res = floor(res);
  return x - (res * y);
#endif
}
