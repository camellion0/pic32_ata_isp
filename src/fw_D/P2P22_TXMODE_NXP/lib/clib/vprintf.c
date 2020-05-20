/*                      - VPRINTF.C -

   The ANSI "vprintf" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdarg.h"
#include "stdio.h"
#include "icclbutl.h"

static void put_one_char(char c, void *dummy)
{
  putchar(c);
  (void)dummy; /* Warning on this line OK (Optimized Away) */
}

int vprintf(const char *format, va_list ap)
{
  return _formatted_write(format, put_one_char, (void *) 0, ap);
}
