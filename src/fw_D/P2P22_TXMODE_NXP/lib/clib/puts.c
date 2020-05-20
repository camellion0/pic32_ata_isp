/*                      - PUTS.C -

   The ANSI "puts" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdio.h"

int puts(const char *s)
{
  while (*s)
  {
    if (!( putchar(*s++)))
      return EOF;
  }
  putchar('\n');
  return 1;
}
