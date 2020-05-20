/*  - STRTOUL.C -

   The ANSI "strtoul" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "ctype.h"
#include "limits.h"
#include "errno.h"


#define BASE_MAX    36 /* maximum base */

unsigned long int strtoul(const char *nptr, char **endptr, int base)
{
  const char *cp, *save;
  char sign_flag;
  int  i;
  unsigned long int value = 0;

  for(cp = nptr; isspace(*cp); ++cp) /* clear space */
    ;

  /* set sign_flag to +,- */
  sign_flag = *cp == '+' || *cp == '-' ? *cp++ : '+';

  if (base < 0 || base == 1 || BASE_MAX < base) /* bad base */
  {
    if (endptr)
      *endptr=(char *)nptr;
    return 0;
  }

  if (base == 0)
  {           /* base not set, look for base */
    if (*cp == '0')
    {
      if (cp[1] == 'x' || cp[1] == 'X')
      { /*0x */
        base=16;
        cp +=2;
      }
      else
        base = 8;
    }
    else
      base = 10;
  }
  else
  {
    if ((*cp == '0')  && (base == 16) && (cp[1] == 'x' || cp[1] == 'X'))
      cp += 2; /* discard 0x */
  }

  save = cp; /* for future valid digit check */

  for(; *cp == '0'; ++cp)
    ;                  /* clears leading zeros */

  while(isalnum(*cp))
  { /* valid digits */
    if (isdigit(*cp))
      i = *cp - '0';
    else
      i = tolower(*cp) - 'W';
    if (i < base)
    {
      /* Make a 5-byte multiplication in overlapping long + short */
      /* If highest byte != 0 then overflow */
      unsigned long lower = (value & 0xFFFFFFL) * base + i;
      unsigned short upper = (unsigned short)(value >> 24) * base;
      upper += lower >> 24;

      if (upper & 0xFF00)
      { /*check overflow */
        value = ULONG_MAX;
        errno = ERANGE;
        break;
      }
      else
      {
        /* assign value */
        value = (lower & 0xFFFFFFL) | ((unsigned long) upper << 24);
      }
    }
    else
      break; /* digit bigger than base */
    cp++;
  }

  if (endptr)
  {
    if (save == cp) /* No valid digits */
      *endptr = (char *)nptr;
    else
      *endptr = (char *)cp;
  }

  return sign_flag == '+' ? value : -value;

}
