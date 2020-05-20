/*                      - SPRINTF_P.C -

   A variant of the ANSI "sprintf" function.

   Version: 1.03 [IANP] (from sprintf 3.00)

*/

#include "stdarg.h"
#include "stdio.h"
#include "iccutl_p.h"

#pragma language=extended

static void put_c_in_string (char c, void *ptr)  /* Low-level output */
  {
    *(*(char **) ptr)++ = c;
  }


int sprintf_P (char *s, PGM_P format, ...)      /* Our main entry */
  {
    va_list ap;
    int nr_of_chars;

    va_start (ap, format);      /* Variable argument begin */
    nr_of_chars = _formatted_write_P(format, put_c_in_string, (void *) &s, ap);
    va_end (ap);                /* Variable argument end */
    *s = '\0';                  /* String should be terminated with NUL */
    return (nr_of_chars);       /* According to ANSI */
  }


