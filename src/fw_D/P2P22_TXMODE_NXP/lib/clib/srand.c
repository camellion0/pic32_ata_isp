/*                      - SRAND.C -

   The ANSI "srand" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"

extern unsigned long int __next_rand;

void srand(unsigned int seed)
{
  __next_rand = seed;
}
