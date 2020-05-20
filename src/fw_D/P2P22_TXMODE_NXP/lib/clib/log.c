/*                      - LOG.C -

   The ANSI "log" function.

   The coefficients are #2705 from Hart & Cheney. (19.38D)

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "iccfloat.h"
#include "float.h"
#include "errno.h"



#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
#define b0      0.20000008368e1
#define b1      0.6664407777
#define b2      0.41517739
#else
#define b0      0.200000000000000261007e1
#define b1      0.6666666666633660894
#define b2      0.400000001206045365
#define b3      0.2857140915904889
#define b4      0.22223823332791
#define b5      0.1811136267967
#define b6      0.16948212488
#endif



double log(double arg)
{
#ifdef _INTRINSIC
  return log(arg);
#else
  double z;
  int exp;

  if (arg <= 0.0)
  {
    if (arg == 0.0)
    {
      errno = ERANGE;
      return -HUGE_VAL;
    }
    errno = EDOM;
    return __EDOM_VALUE;
  }

  /* Using log(x) = log(2^n * y) = log(2^n) + log(y) =    */
  /* n * log(2) + log(y), where 1/sqrt(2) <= y <= sqrt(2) */

  arg = frexp(arg,&exp);

#ifdef __IAR_SYSTEMS_ICC__
  if (arg < __SQRTO2)
  {
    arg = __daddexp(arg,1);
    exp--;
  }
#else
  while (arg < 0.5)
  {
    arg *= 2;
    exp--;
  }
  if (arg < __SQRTO2)
  {
    arg *= 2;
    exp--;
  }
#endif

  arg = (arg - 1) / (arg + 1);
  z = arg * arg;

#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
  arg = arg * ((b2 * z + b1) * z + b0);
  return (signed char) exp * __LOG2 + arg;
#else
  arg = arg * ((((((b6 * z + b5) * z + b4) * z + b3) * z + b2) * z + b1) *
               z + b0);
  return exp * __LOG2 + arg;
#endif
#endif
}
