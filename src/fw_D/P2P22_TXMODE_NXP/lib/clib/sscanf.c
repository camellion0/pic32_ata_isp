/*                      - SSCANF.C -

   The ANSI "sscanf" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdarg.h"
#include "stdlib.h"
#include "stdio.h"
#include "icclbutl.h"   /* Contains low-level declarations */

int sscanf(const char *line, const char *format,...)
{
  va_list ap;
  int conversions;

  if (*line)
  {
    va_start(ap, format);
    conversions = _formatted_read(&line, &format, ap);
    va_end(ap);
    return conversions;
  }
  else
  {
    return EOF;
  }
}
