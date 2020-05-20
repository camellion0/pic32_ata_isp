/*                      - SCANF_P.C -

   A variant of the ANSI "scanf" function

   Version: 1.03 [IANP] (from scanf.c 3.10)
          
*/

#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include "iccutl_p.h" /* Contains low-level declarations */

#ifndef SCANF_MAXLINE
#define SCANF_MAXLINE 128
#endif

#ifndef MEM_ATTRIBUTE
#define MEM_ATTRIBUTE
#endif

#pragma language=extended

int scanf_P (PGM_P format,...)
{
  MEM_ATTRIBUTE char line_buffer[SCANF_MAXLINE];
  char *cp;
  va_list ap;
  int conversions = 0;
  
  if (gets (cp = line_buffer))
  {
    va_start (ap, format);
    do
    {
      if (*cp)
      { 
        conversions += _formatted_read_P ((const char **)&cp, &format, ap);
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
    va_end (ap);
    return (conversions);
  }
  else
  {
    return (EOF);
  }
}
