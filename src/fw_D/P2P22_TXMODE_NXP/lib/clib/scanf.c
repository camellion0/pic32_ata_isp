/*                      - SCANF.C -

   The ANSI "scanf" function

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include "icclbutl.h"   /* Contains low-level declarations */

#ifndef SCANF_MAXLINE
#define SCANF_MAXLINE 128
#endif

#ifndef MEM_ATTRIBUTE
#define MEM_ATTRIBUTE
#endif

int scanf (const char *format,...)
{
  MEM_ATTRIBUTE char line_buffer[SCANF_MAXLINE];
  char *cp;
  va_list ap;
  int conversions = 0;

  if (gets (cp = line_buffer))
  {
    va_start(ap, format);
    do
    {
      if (*cp)
      {
        conversions += _formatted_read((const char **)&cp, &format, ap);
      }
      if (*format && !*cp)
      {
        if (!gets(cp = line_buffer))
        {
          line_buffer[0] = '\0';
        }
      }
    }
    while (*format);

    va_end(ap);
    return conversions;
  }
  else
  {
    return EOF;
  }
}
