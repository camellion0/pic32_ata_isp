/*                      - TOUPPER.C -

   The ANSI "toupper" function (also available as a macro).

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

#ifdef toupper
#undef toupper
#endif

int toupper(int c)
{
#ifdef _INTRINSIC
  return toupper(c);
#else
  return (c >= 'a' &&  c <= 'z') ? c ^ 0x20 : c;
#endif
}
