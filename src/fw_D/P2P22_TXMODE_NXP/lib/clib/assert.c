/*                      - ASSERT.C -

   _Assert is called by the assert macro in case the assertion is false.

   NOTE: According to ANSI the information should be reported to 'stderr'
   for hosted compilers, for freestanding (cross) compilers as ICC ANSI says
   nothing about where to report.  Here we just give the information to
   to stdout via printf().

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include <stdio.h>      /* declaration of printf () */
#include <stdlib.h>     /* declaration of abort ()  */


void _Assert(const char *arg, const char *filename, int lineno)
{
  printf("Assertion failed: %s, file %s, line %d\n",
         arg, filename, lineno);
  abort();
}
