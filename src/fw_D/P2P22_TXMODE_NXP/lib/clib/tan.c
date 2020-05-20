/*                      - TAN.C -

   The ANSI "tan" function.

   A series is used after range reduction.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "iccfloat.h"

#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
#define t0      0.211849369664121e3
#define t1      -0.125288887278448e2
#define s0      0.2697350131214121e3
#define s1      -0.714145309347748e2
#define s2      0.1e1
#else
#define t0      -0.16045331195592187943926861e5
#define t1      0.1269754837658082883786072e4
#define t2      -0.17135185514886110932101e2
#define t3      0.282087729716551031514e-1
#define s0      -0.20429550186600697853114142e5
#define s1      0.5817359955465568673903419e4
#define s2      -0.181493103540890459934575e3
#define s3      0.1e1
#endif

double tan(double arg)
{
#ifdef _INTRINSIC
  return tan(arg);
#else
  double xsq;
  unsigned char flag,sign;

  flag = 0;
  sign = arg < 0.0;
  arg = fabs(arg);
  arg *= (4.0 * __INVPI);
  arg = modf(arg, &xsq); /* Overflow ? */
  switch ((unsigned char) xsq % 4)
  {
  case 1:
    arg = 1.0 - arg;
    flag = 1;
    break;
  case 2:
    sign = !sign;
    flag = 1;
    break;
  case 3:
    arg = 1.0 - arg;
    sign = !sign;
    break;
  case 0:
    break;
  }

  xsq = arg * arg;
#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
  arg = arg * ((t1 * xsq + t0) / ((s2 * xsq + s1) * xsq + s0));
#else
  arg = arg * ((((t3 * xsq + t2) * xsq + t1) * xsq + t0) /
               (((s3 * xsq + s2) * xsq + s1) * xsq + s0));
#endif

  if (flag)
    arg = 1.0 / arg;
  return sign ? -arg : arg;
#endif
}
