/*                      - FLOOR.C -

   The ANSI "floor" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"
#include "iccfloat.h"


double floor(double d)
{
#ifdef _INTRINSIC
  return floor(d);
#else
  double e;

  if (fabs(d) >= 1.0e7)
    return d;

  if (d < 0.0)
  {
    d = fabs(d);
    e = (long) d;
    if (d - e)
      e += 1;
    return -e;
  }
  else
    return (long) d;
#endif
}
