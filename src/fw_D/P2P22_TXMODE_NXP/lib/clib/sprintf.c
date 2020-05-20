/*                      - SPRINTF.C -

   The ANSI "sprintf" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdarg.h"
#include "stdio.h"
#include "icclbutl.h"

static void put_c_in_string(char c, void *ptr)  /* Low-level output */
{
  *(*(char **) ptr)++ = c;
}


int sprintf(char *s, const char *format, ...)          /* Our main entry */
{
  va_list ap;
  int nr_of_chars;

  va_start(ap, format);      /* Variable argument begin */
  nr_of_chars = _formatted_write(format, put_c_in_string, (void *) &s, ap);
  va_end(ap);                /* Variable argument end */
  *s = '\0';                 /* String should be terminated with NUL */
  return nr_of_chars;        /* According to ANSI */
}
