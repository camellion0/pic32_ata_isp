/*      - STRTOD.C -

   The ANSI "strtod" function.


   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"
#include "ctype.h"
#include "math.h"
#include "float.h"
#include "errno.h"
#include "limits.h"

#define MAX_DIG LDBL_DIG


static double ipow(short y)
{
  unsigned short N;
  long double result, x = 10.0;

  /*==============================================*/
  /* Calculate mant ^ I using the 'binary method' */
  /* See Knuth Vol 2 p 441 ->         */
  /* The result will be in 'result'       */
  /*==============================================*/
  if (y == 0.0)
    return 1.0;
  if (y == 1.0)
    return 10.0;

  for (result = 1.0, N = abs(y); ; N /= 2)
  {
    if (N & 0x0001)
    {
      result *= x;
    }
    if (N == 1)     /* Avoid one extra multiplication */
      break;
    x *= x;
  }

  if (y < 0.0)
    result = 1.0 / result;

  return result;
}



/*======================================================================*/
/*                                                                      */
/*    Convert a string to a doubble. Reads data from "*cp".             */
/*                                                                      */
/*  Returns: The converted value.                                       */
/*                                                                      */
/*======================================================================*/

double strtod(const char *nptr, char **endptr)
{
  const char *cp, *save_cp;
  char sign_flag, exp_sign;
  char found_zeros = 0;
  short  main_dig = 0, float_dig = 0, exponent = 0;
  long double fvalue = 0, factor;

/*======================================================================*/
/*    Read digits                                                       */
/*======================================================================*/
  for(cp = nptr; isspace(*cp); ++cp)
    ; /* remove space */

  /* sets sign_flag to + or - */
  sign_flag = *cp == '-' || *cp == '+'? *cp++ : '+';

  for(; *cp == '0'; ++cp)
    found_zeros = 1;      /* clears beginning zeros */

  while (isdigit(*cp))
  {  /* stores main digits in fvalue */
    if (++main_dig <= MAX_DIG) /* only store until max digits, discard rest */
      fvalue = fvalue * 10 + (*cp - '0');
    cp++;
  }

  if (main_dig >= DBL_MAX_10_EXP)
    goto BAD_CONV_HUGE;

  float_dig += main_dig;

  if (*cp == '.')
  {
    cp++;
    while (isdigit (*cp))
    { /* stores float digits in fvalue until max digits */
      if (float_dig < MAX_DIG)
      {
        float_dig++;
        fvalue = fvalue * 10 + (*cp - '0');
      }
      cp++; /* discard rest */
    }
  }

  float_dig -= main_dig;
  save_cp = cp;

  if ((*cp == 'e') || (*cp == 'E'))
  {
    *cp++;
    exp_sign = *cp == '-' || *cp == '+'? *cp++ : '+'; /* sets exp_sign */
    if (!isdigit (*cp))
      cp = save_cp; /* No exponent */
    else
    {               /* exponent conatins value */
      for (;
           isdigit(*cp) && (exponent <= DBL_MAX_10_EXP);
           exponent = exponent * 10 + (*cp++ - '0'))
        ;
      for(; isdigit(*cp); cp++)
        ; /*discards remaining digits */
    }
  }

/*==========================================================================*/
/*     Range Check                                                          */
/*==========================================================================*/

  exponent += --main_dig; /* raise exponent by main digits -1 */

  if (exponent > DBL_MAX_10_EXP)
  {  /* To big or small value */
    if (exp_sign == '+')
      goto BAD_CONV_HUGE;
    else
    {
      errno = ERANGE;
      goto BAD_CONV_NULL;
    }
  }


  factor = fvalue;   /* assign fvalue converted value */

  exponent = exp_sign == '+'? exponent - main_dig - float_dig:
                              main_dig - exponent - float_dig;
  fvalue *= ipow(exponent);


  if (factor && !fvalue)
    goto BAD_CONV_NULL;

  if ((fvalue >=0.0) && (fvalue > DBL_MAX)) /* to big */
    goto BAD_CONV_HUGE;
  else
    {
      if (factor && (fvalue < DBL_MIN))
        goto BAD_CONV_NULL;
    }

  if (!(++main_dig) && !float_dig && !found_zeros) /* no digits */
  {
    if (endptr)
      *endptr = (char *)nptr;
    return 0.0;
  }

  if (endptr)
    *endptr = (char *)cp;

  return (double)(sign_flag == '+' ? fvalue : -fvalue);

/*======================================================================*/
/*  Not a good conversion char, return                                  */
/*======================================================================*/
BAD_CONV_NULL:
  if (endptr)
    *endptr = (char *)nptr;
  errno = ERANGE;
  return 0;

BAD_CONV_HUGE:
  if (endptr)
    *endptr = (char *)nptr;
  errno = ERANGE;
  return sign_flag == '+' ? HUGE_VAL : -HUGE_VAL;
}
