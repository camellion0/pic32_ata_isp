/*                      - STRNCAT.C -

   The ANSI "strncat" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

char *strncat(char *s1, const char *s2, size_t n)
{
  char *os1 = s1;

  while (*s1++)
    ;
  --s1;
  while (*s1++ = *s2++)
  {
    if (!n--)
    {
      *--s1 = '\0';
      break;
    }
  }
  return os1;
}
