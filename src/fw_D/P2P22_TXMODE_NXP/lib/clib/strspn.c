/*                      - STRSPN.C -

   The ANSI "strspn" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

size_t strspn(const char *s1, const char *s2)
{
#ifdef _INTRINSIC
  return strspn(s1, s2);
#else
  char* v;
  size_t n = 0;

  while (*s1)
  {
    v = (char *)s2;
    while (*v)
      if (*v++ == *s1)
      {
        n++;
        goto OK;
      }
    break;
OK:
    s1++;
  }
  return n;
#endif
}
