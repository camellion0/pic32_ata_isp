/*                      - CEIL.C -

   The ANSI "ceil" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "math.h"
#include "iccfloat.h"

double ceil(double d)
{
#ifdef _INTRINSIC
  return ceil(d);
#else
  return -floor(-d);
#endif
}
