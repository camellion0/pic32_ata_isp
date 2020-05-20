/*                      - MEMCHR.C -

   The ANSI "memchr" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

void *memchr(const void *s, int c, size_t n)
{
#ifdef _INTRINSIC
  return memchr(s, c, n);
#else
  while (n--)
  {
    if ( (unsigned char)c == *(unsigned char*)s)
    {
      return (void *)s;
    }
    s = ((char*)s) + 1;
  }
  return NULL;
#endif
}
