/*                      - LABS.C -

   The ANSI "labs" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"

long int labs(long int j)
{
#ifdef _INTRINSIC
  return labs(j);
#else
  return j >= 0 ? j : -j;
#endif
}
