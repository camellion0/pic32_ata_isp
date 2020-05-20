/*                      - STRCHR.C -

   The ANSI "strchr" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

char *strchr(const char *s, int c)
{
#ifdef _INTRINSIC
  return strchr(s, c);
#else
  do
  {
    if (*s == (char)c)
    {
      return (char *)s;
    }
  }
  while (*s++);

  return NULL;
#endif
}
