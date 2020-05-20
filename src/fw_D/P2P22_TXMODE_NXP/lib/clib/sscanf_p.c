/*                      - SSCANF_P.C -

   A variant of the ANSI "sscanf" function.

   Version: 1.03 [IANP] (from sscanf 3.00)

*/

#include "stdarg.h"
#include "stdlib.h"
#include "stdio.h"
#include "iccutl_p.h" /* Contains low-level declarations */

#pragma language=extended

int sscanf_P (const char *line, PGM_P format,...)
  {
    va_list ap;
    int conversions;
  
    if (*line)
      {
        va_start (ap, format);
        conversions = _formatted_read_P (&line, &format, ap);
        va_end (ap);
        return (conversions);
      }
    else
      {
        return (EOF);
      }
  }
