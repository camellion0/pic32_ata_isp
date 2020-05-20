/*                      - ATOF.C -

   The ANSI "atof" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "ctype.h"
#include "stdlib.h"

double atof(const char *s)
{
#ifdef _INTRINSIC
  return atof(s);
#else
  double val, power;
  int exp;
  unsigned char sign = 0;

  while (isspace(*s))
  {
    s++;
  }

  if (*s == '-')
  {
    s++;
    sign = 1;
  }
  else if (*s == '+')
  {
    s++;
  }

  val = 0;
  while (isdigit(*s))
  {
    val = val * 10 + (*s++ - '0');
  }

  power = 1;

  if (*s == '.')
  {
    while (isdigit(*++s))
    {
      val = val * 10 + (*s - '0');
      power = power * 10;
    }
  }

  if (tolower(*s) == 'e')
  {
    exp = atoi(++s);
    while (exp < 0)
    {
      val /= 10;
      exp++;
    }
    while (exp > 0)
    {
      val *= 10;
      exp--;
    }
  }

  val /= power;
  return sign ? -val : val;
#endif
}
