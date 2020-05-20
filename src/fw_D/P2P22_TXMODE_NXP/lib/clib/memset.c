/*                      - MEMSET.C -

   The ANSI "memset" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

void *memset(void *s, int c, size_t n)
{
#ifdef _INTRINSIC
  return memset(s, c, n);
#else
  char *v;

  v = s;
  while (n--)
  {
    *v++ = c;
  }
  return s;
#endif
}
