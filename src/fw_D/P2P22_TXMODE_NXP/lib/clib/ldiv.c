/*                      - LDIV.C -

   The ANSI "ldiv" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"

ldiv_t ldiv(long int numer, long int denom)
{
#ifdef _INTRINSIC
  return ldiv(numer, denom);
#else
  ldiv_t lres;

  lres.quot = numer / denom;
  lres.rem = numer - lres.quot * denom;
  return lres;
#endif
}
