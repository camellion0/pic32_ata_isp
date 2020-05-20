/*                      - RAND.C -

   The ANSI "rand" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"

unsigned long int __next_rand = 1;

int rand(void)
{
  __next_rand = __next_rand * 1103515245 + 12345;
  return (unsigned int) (__next_rand >> 16) & RAND_MAX;
}
