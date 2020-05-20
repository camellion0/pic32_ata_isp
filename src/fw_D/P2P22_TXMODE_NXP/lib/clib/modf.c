/*                      - MODF.C -

   The ANSI "modf" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"
#include "iccfloat.h"

double modf(double value, double *iptr)
{
#ifdef _INTRINSIC
  return modf(value, iptr);
#else
  double temp;

  if (fabs(value) >= 1.0e7)
  {
    *iptr = value;
    return 0;
  }
  else
  {
    *iptr = temp = (double) (long) value;
    return value - temp;
  }
#endif
}
