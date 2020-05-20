/*                      - PRINTF_P.C -

   A variant of the ANSI "printf" function.

   Version: 1.03 [IANP] (from printf 3.00)

*/

#include "stdarg.h"
#include "stdio.h"
#include "iccutl_p.h"

#pragma language=extended

static void put_one_char(char c, void *dummy)
  {
    putchar (c);
  }


int printf_P (PGM_P format, ...)                        /* Our main entry */
  {
    va_list ap;
    int nr_of_chars;

    va_start (ap, format);      /* Variable argument begin */
    nr_of_chars = _formatted_write_P (format, put_one_char, (void *) 0, ap);
    va_end (ap);                /* Variable argument end */
    return (nr_of_chars);       /* According to ANSI */
  }

