/*                      - POW.C -

   The ANSI "pow" functions.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "stdlib.h"
#include "iccfloat.h"
#include "float.h"
#include "limits.h"
#include "errno.h"


double pow(double x,double y)
{
#ifdef _INTRINSIC
  return pow(x, y);
#else
  unsigned short N;
  short I;
  short negate = 0;
  double mantissa;
  double exponent;
  double result;
  double dI,ey,frac;

  /*=====================*/
  /* First special cases */
  /*=====================*/

  if (y == 0.0)
    return 1.0;

  if (x == 0.0)
    return 0.0;

  if (fabs(x) == HUGE_VAL)
  {
    errno = ERANGE;
    return x;
  }

  if (y == HUGE_VAL)
  {
    errno = ERANGE;
    return HUGE_VAL;
  }

  if (y == -HUGE_VAL)
    return 0.0;

  if (x < 0.0)
  {
    /*======================================*/
    /* Check if y is not an integral value  */
    /* and in that case rise EDOM           */
    /*======================================*/
#if __FLOAT_SIZE__ != __DOUBLE_SIZE__
    if (y > ULONG_MAX)
    {
      if (modf(y, &mantissa) > 2 * DBL_EPSILON)
      {
        goto MY_COMMON_SUBEXPR_OPTIM;
      }
    }
    else
#endif
      if ( (y - (long)y) > 2 * DBL_EPSILON)
      {
#if __FLOAT_SIZE__ != __DOUBLE_SIZE__
MY_COMMON_SUBEXPR_OPTIM:
#endif
        errno = EDOM;
        return 0.0;
      }

    negate = ((unsigned long) y) & 1;
    x = fabs(x);
  }

  /*-----------------------------------------------------------*/
  /* x ^ y = [x = mant * 2 ^ exp] = mant ^ y * 2 ^ (exp * y) = */
  /* [exp * y = I + frac; I < DBL_EXP_MAX] =                   */
  /* (mant ^ I) * (2 ^ I) * (mant ^ (y-I) * 2 ^frac)           */
  /*-----------------------------------------------------------*/

  /*============================*/
  /* Rewrite x = mant * 2 ^ exp */
  /*============================*/

  mantissa = __dnormexp(x);
  exponent  = __dgetexp(x);

  ey = y * exponent;

  /*============================*/
  /* Rewrite y * exp = I + frac */
  /*============================*/

  dI = floor(ey);
  I = (short) dI;
  frac = ey - dI;

  /*============================================*/
  /* First calculate  'mant ^ (y-I) * 2 ^ frac' */
  /*============================================*/

  result = exp((y - I) * log(mantissa) + frac * __LOG2);

  /*==============================================*/
  /* Calculate mant ^ I using the 'binary method' */
  /* See Knuth Vol 2 p 441 ->                     */
  /* The result will be in 'frac'                 */
  /*==============================================*/
  frac = 1.0;

  if ((N = abs(I)) != 0)             /* no else needed - frac is 1.0 */
  {
    for (; ; N /= 2)
    {
      if (N & 0x0001)
      {
        frac *= mantissa;
      }
      if (N == 1)               /* Avoid one extra multiplication */
        break;
      mantissa *= mantissa;
    }

    if (I < 0)
      frac = 1.0 / frac;
  }

  /*================================*/
  /* And multiply that with '2 ^ I' */
  /*================================*/

  mantissa = __daddexp(frac, I);

  result *= mantissa;

  return (negate ? -result : result);
#endif
}


#ifdef SELF_TEST

#include <stdio.h>

void main(void)
{
  long j;

  if ((j = pow (0.23, 7.021) * 1E8) != 3301)
    printf ("error pow(0.23, 7.021) %ld [3301]\n",j);
  if ((j = pow (4711.023, -5.0) * 1E22) != 4309)
    printf ("error pow(4711.023, -5.0) %ld [4309]\n",j);
  if ((j = pow (1.5685, -18.2941) * 1E6) != 265)
    printf ("error pow(1.5685, -18.2941) %ld [265]\n",j);
  if ((j = pow (0.1219, -7.721) * 1E-3) != 11401)
    printf ("error pow(0.1219, -7.721) %ld [11401]\n",j);
  if ((j = pow (-0.1219, 7.0) * 1E10) != -3999)
    printf ("error pow(-0.1219, 7.0) %ld [3999]\n",j);
  if ((j = pow (2.0001, 12.0) ) != 4098)
    printf ("error pow(2.0001, 12.0) %ld [4098]\n",j);
  if ((j = pow (10.0, -HUGE_VAL) ) != 0)
    printf ("error pow(10.0, -HUGE_VAL) %ld [0]\n",j);
  if (( pow (10.0, HUGE_VAL) ) != HUGE_VAL)
    printf ("error pow(10.0, HUGE_VAL) != HUGE_VAL\n");
  if ((j = pow (-3.0, 3.0) ) != -27)
    printf ("error pow(-3.0, 3.0) %ld [-27]\n",j);
  pow (-4.3, 1.1);
  if (errno != EDOM)
    printf ("error pow(-10.0, 3.3) should set errno %d [EDOM]\n",errno);
}
#endif
