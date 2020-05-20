/*                      - ABS.C -

   The ANSI "abs" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"

int abs(int j)
{
#ifdef _INTRINSIC
  return abs(j);
#else
  return j >= 0 ? j : -j;
#endif
}
