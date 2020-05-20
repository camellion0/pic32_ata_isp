/*                      - MEMCPY.C -

   The ANSI "memcpy" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

void *memcpy(void *s1, const void *s2, size_t n)
{
#ifdef _INTRINSIC
  return memcpy(s1,s2,n);
#else
  char *v;

  v = (char*) s1;
  while (n--)
  {
    *v++ = *(char*)s2;
    s2 = (char*)s2 + 1;
  }
  return s1;
#endif
}
