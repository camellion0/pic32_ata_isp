/*  - STRXFRM.C -

   The ANSI "strxfrm" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

size_t strxfrm(char *s1, const char *s2, size_t n)
{
  strncpy(s1,s2,n);
  return strlen(s2);
}
