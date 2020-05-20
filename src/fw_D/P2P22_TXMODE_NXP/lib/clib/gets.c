/*                      - GETS.C -

   The ANSI "gets" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdio.h"

char *gets(char *s)
{
  char *ptr;
  int  c;

  ptr = s;
  while ((c = getchar()) != '\n' && c != EOF)
  {
    *ptr++ = c;
  }

  if (c == EOF && ptr == s)
  {
    return 0;
  }

  *ptr = '\0';
  return s;
}
