/*                      - MEMCMP.C -

   The ANSI "memcmp" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

int memcmp(const void *s1, const void *s2, size_t n)
{
#ifdef _INTRINSIC
  return memcmp(s1, s2, n);
#else
  while (n && *(char*)s1 == *(char*)s2)
  {
    s1 = (char*)s1 + 1;
    s2 = (char*)s2 + 1;
    n--;
  }
  return  n ? *(unsigned char *)s1 - *(unsigned char *)s2 : 0;
#endif
}
