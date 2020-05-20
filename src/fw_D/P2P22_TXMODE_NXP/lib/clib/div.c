/*                      - DIV.C -

   The ANSI "div" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"

div_t div(int numer, int denom)
{
#ifdef _INTRINSIC
  return div(numer, denom);
#else
  div_t res;

  res.quot = numer / denom;
  res.rem = numer - res.quot * denom;
  return res;
#endif
}
