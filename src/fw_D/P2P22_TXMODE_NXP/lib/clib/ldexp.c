/*                      - LDEXP.C -

   The ANSI "ldexp" function.

$Revision: 328482 $

*/

#include "math.h"
#include "iccfloat.h"
#include "float.h"
#include "errno.h"

double ldexp(double value, int exp)
{
#ifdef _INTRINSIC
  return ldexp(value, exp);
#else
  short check;

  if (value == 0.0)
    return 0.0;

  check = __dgetexp(value) + exp;
  if ((exp < 2 * DBL_MIN_EXP) || (check < DBL_MIN_EXP))
  {
    errno = ERANGE;
    /*================================*/
    /* Check for wrap-around in check */
    /*================================*/
    if (exp > 2 * DBL_MAX_EXP)
      return (value < 0.0) ? -HUGE_VAL : HUGE_VAL;
    return 0.0;
  }
  if ((exp > 2 * DBL_MAX_EXP) || (check > DBL_MAX_EXP))
  {
    errno = ERANGE;
    return (value < 0.0) ? -HUGE_VAL : HUGE_VAL;
  }
  value = __daddexp(value,exp);
  return value;
#endif
}

#ifdef SELF_TEST

#include <stdio.h>

void main(void)
{
  long j;

  if (ldexp(1e37, 32767) != HUGE_VAL)
    printf ("error ldexp(1e37,32767) [HUGE_VAL]\n");
}
#endif
