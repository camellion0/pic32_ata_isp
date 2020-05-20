/*                      - TOLOWER.C -

   The ANSI "tolower" function (also available as a macro).

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

#ifdef tolower
#undef tolower
#endif

int tolower(int c)
{
#ifdef _INTRINSIC
  return tolower(c);
#else
  return (c >= 'A' &&  c <= 'Z') ? c | 0x20 : c;
#endif
}
