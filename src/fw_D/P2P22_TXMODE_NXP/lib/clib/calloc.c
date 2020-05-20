/*                      - CALLOC.C -

   The ANSI "calloc" function.

   Assumes that storage is reserved in a linear place.
   Should be completely CPU-independent.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"

void *calloc(size_t k, size_t s)
{
  char *p, *p1;
  size_t n = k * s;

  if (p1 = p = malloc(n))   /* There is available memory */
  {
    while (n--)
      *p1++ = 0;                     /* Zero out area */
  }
  return p;
}
