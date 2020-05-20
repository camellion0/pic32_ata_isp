/*                      - TANH.C -

   The ANSI "tanh" function.


   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "float.h"

double tanh (double arg)
{
#ifdef _INTRINSIC
  return tanh(arg);
#else
  double absarg;
  unsigned char sign;

  sign = arg < 0.0;
  absarg = fabs(arg);

  if (absarg < DBL_MIN)
    arg = 0.0;
#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
  else if (arg < 8.32 )       /* arg < (n+1)/2 ln 2 */
#else
  else if (arg < 18.72)
#endif
  {
    arg = exp(2.0 * absarg);
    arg = (arg - 1.0) / (arg + 1.0);
  }
  else
    arg = 1.0;
  return sign ? -arg: arg;
#endif
}
