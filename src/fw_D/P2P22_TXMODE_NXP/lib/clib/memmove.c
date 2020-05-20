/*                      - MEMMOVE.C -

   The ANSI "memmove" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/


#include "string.h"

void *memmove(void *s1, const void *s2, size_t n)
{
#ifdef _INTRINSIC
  return memmove(s1, s2, n);
#else
  void *v;

  v = s1;
  if ( (char*)s2 <= (char*)s1  &&  ((char*)s2 + n) >= (char*)s1 )
  {
    s1 = (char*)s1 + n;
    s2 = (char*)s2 + n;
    while (n--)
    {
      s2 = (char*)s2 - 1;
      s1 = (char*)s1 - 1;
      *(char*)s1 = *(char*)s2;
    }
  }
  else
  {
    while (n--)
    {
      *(char*)s1 = *(char*)s2;
      s2 = (char*)s2 + 1;
      s1 = (char*)s1 + 1;
    }
  }
  return v;
#endif
}
