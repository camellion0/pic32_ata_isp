/*                      - STRRCHR.C -

   The ANSI "strrchr" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"

char *strrchr(const char *s, int c)
{
#ifdef _INTRINSIC
  return strrchr(s, c);
#else
  char *v = NULL;

  while (*s)
  {
    if (*s == (char)c)
      v = (char *)s;
    s++;
  }
  return v;
#endif
}
