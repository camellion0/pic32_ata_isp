/*                      - STRCOLL.C -

   The ANSI "strcoll" function.  Note: without "locale" == strcmp.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

int strcoll(const char *s1, const char *s2)
{
  return strcmp(s1, s2);
}
