/*                      - EXP.C -

   The ANSI "exp" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "iccfloat.h"
#include "float.h"
#include "errno.h"

#if __FLOAT_SIZE__ == __DOUBLE_SIZE__

#define   maxf     88

#define b6      0.217022554700507e-3
#define b5      0.1243968782097219e-2
#define b4      0.9678840997260719e-2
#define b3      0.55483341983730451e-1
#define b2      0.24022983627421589
#define b1      0.693146983840595958
#define b0      0.100000000185579975e1


double exp(double arg)
{
#ifdef _INTRINSIC
  return exp(arg);
#else
  double xsq;
  signed char ent;
  unsigned char flag;

  if (arg == 0.0)
    return 1.0;
  if (fabs(arg) > maxf)
  {
    errno = ERANGE;
    return arg < 0.0 ? 0.0 : HUGE_VAL;
  }

  /* Using exp(x) = 2^(x / log(2)) =          */
  /* 2^(n + y) = 2^(n + (y + 0.5) - 0.5) =    */
  /* 2^n * 2^y = 2^n * 2^(y + 0.5) * 2^-0.5   */
  /* where n is integral and 0.5 <= y < 1     */

  /* Scale the argument */
  arg *= __LOG2E;

  /* Get integral */
  ent = (signed char)fabs(arg);
  if (arg < 0.0)
  {
    if (arg - ent)
      ent++;
    ent = -ent;
  }

  /* Get fraction */
  xsq = arg - ent;
  if (xsq < 0.5)
  {
    xsq += 0.5;
    flag = 1;
  }
  else
    flag = 0;

  arg = (((((b6 * xsq + b5) * xsq + b4) * xsq + b3) * xsq + b2) * xsq +
         b1) * xsq + b0;

  if (flag)
    arg *= (1.0 / __SQRT2);

  /* This is safe because -88 <= ent <= 88 (float),           */
  /* -1000 <= ent <= 1000 (double), and 1.0 <= arg <= 2.0 */
#ifdef __IAR_SYSTEMS_ICC__
  return __daddexp(arg,ent);
#else
  return ldexp(arg, ent);
#endif
#endif
}


#else           /* 'double' i.e. 64 bit floats  */

#define   maxf     1000

#define p0      0.1513906799054338915894328E04
#define p1      0.20202065651286927227886E02
#define p2      0.23093347753750233624E-01
#define q0      0.4368211662727558498496814E04
#define q1      0.233184211427481623790295E03
#define q2      0.1E01


double exp(double arg)
{
#ifdef _INTRINSIC
  return exp(arg);
#else
  double x,xsq,P,Q;
  signed short ent;
  unsigned char flag;

  if (arg == 0.0)
    return 1.0;
  if (fabs(arg) > maxf)
  {
    errno = ERANGE;
    return arg < 0.0 ? 0.0 : HUGE_VAL;
  }

  /* Using exp(x) = 2^(x / log(2)) =          */
  /* 2^(n + y) = 2^(n + (y + 0.5) - 0.5) =    */
  /* 2^n * 2^y = 2^n * 2^(y + 0.5) * 2^-0.5   */
  /* where n is integral and 0.5 <= y < 1     */

  /* Scale the argument */
  arg *= __LOG2E;

  /* Get integral */
  ent = (signed short)fabs(arg);
  if (arg < 0.0)
  {
    if (arg - ent)
      ent++;
    ent = -ent;
  }

  /* Get fraction */
  x = arg - ent;
  if (x > 0.5)
  {
    x -= 0.5;
    flag = 1;
  }
  else
    flag = 0;

  xsq = x * x;

  Q = (q2 * xsq + q1) * xsq + q0;

  P = ((p2 * xsq + p1) * xsq + p0) * x;

  arg = (Q+P)/(Q-P);

  if (flag)
    arg *= __SQRT2;

  /* This is safe because -88 <= ent <= 88 (float),           */
  /* -1000 <= ent <= 1000 (double), and 1.0 <= arg <= 2.0 */
#ifdef __IAR_SYSTEMS_ICC__
  return __daddexp(arg,ent);
#else
  return ldexp(arg, ent);
#endif
#endif
}

#endif
