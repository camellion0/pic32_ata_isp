/*                      - SQRT.C -

   The ANSI "sqrt" function.

   Newton's method.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "iccfloat.h"
#include "float.h"
#include "errno.h"

double sqrt(double arg)
{
#ifdef _INTRINSIC
  return sqrt(arg);
#else
  double x,y;
  int exp;

  /* Check the argument */
  if (arg <= 0.0)
  {
    if (arg == 0.0)
      return 0.0;
    errno = EDOM;
    return __EDOM_VALUE;
  }

  /* We use sqrt(x) = 2^k * sqrt(x * 2^-2k) = */
  /* 2^k * sqrt(2) * sqrt(x * 2^-2(k + 1))    */

  /* Divide the argument into 0.5 <= x < 1.0, 2^exp */
  x = frexp(arg,&exp);

  /* Get a start approximation */
  y = 0.57155 * (x + 0.75787);

  /* First half of a Heron iteration */
  y = y + x / y;

  /* Choose if using first or second equivalence above */
  if (exp & 1)
  {
    y *= __SQRT2;
#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
    x = __daddexp(y,
                  (signed char) ((signed char) ((signed char) exp - 1) / 2));
#else
    x = __daddexp(y,(exp - 1) / 2);
#endif
  }
  else
#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
    x = __daddexp(y,(signed char) ((signed char) exp / 2));
#else
    x = __daddexp(y,exp / 2);
#endif

  /* Second half of a Heron iteration */
  x = 0.25 * x + arg / x;

#if __FLOAT_SIZE__ != __DOUBLE_SIZE__
  /* A Newton iteration, if double precision */
  x = 0.5 * (x + arg / x);
#endif
  return x;
#endif
}
