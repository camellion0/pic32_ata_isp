/*                      - STRLEN.C -

   The ANSI "strlen" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

size_t strlen(const char *s)
{
#ifdef _INTRINSIC
  return strlen(s);
#else
  char const *p = s;

  while (*p)
    p++;
  return p - s;
#endif
}
